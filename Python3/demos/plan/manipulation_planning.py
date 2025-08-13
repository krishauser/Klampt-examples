from __future__ import annotations
from klampt.plan import robotcspace,cspace,robotplanning
from klampt.plan import multistep
from klampt.model.robotinfo import RobotInfo
from klampt.model.gripperinfo import GripperInfo
from klampt.math import se3,so3,vectorops
from klampt import vis 
from klampt.io import resource
from klampt.model import ik
from klampt.model import trajectory,cartesian_trajectory
from klampt.model.collide import WorldCollider
from klampt import *
from typing import Dict,List,Tuple,Union,Optional
from klampt.model.typing import Config,Configs,RigidTransform
from dataclasses import dataclass,field
import time
import sys

#Default motion planner settings
#set up a settings dictionary here.  This is a random-restart + shortcutting
#SBL planner.
PLANNER_SETTINGS = { 'type':"sbl", 'perturbationRadius':0.5, 'bidirectional':1, 'shortcut':1, 'restart':1, 'restartTermCond':"{foundSolution:1,maxIters:1000}" }
#You might edit the values 500 and 10 to play with how many iterations /time 
#to give the planner.
MAX_PLANNER_ITERS = 500
MAX_PLANNER_TIME = 10.0
EDGE_CHECK_RESOLUTION = 0.01

#Setup settings: feel free to edit these to see how the results change
#Simplify geometry or not?
SIMPLIFY_TYPE = None
#SIMPLIFY_TYPE = 'aabb'
#SIMPLIFY_TYPE = 'ConvexHull'
DEBUG_SIMPLIFY = 0



@dataclass
class PickAndPlaceContext:
    """A context for pick and place planning, containing any non-variable
    items. Currently only stores the world, robot, gripper info, and planner settings."""
    world: WorldModel
    robot: RobotModel
    gripper: GripperInfo
    object: RigidObjectModel
    startConfig : Config = None
    startObjectConfig : RigidTransform = None
    planner_settings : dict = field(default_factory=lambda : PLANNER_SETTINGS.copy())
    edge_check_resolution :float = EDGE_CHECK_RESOLUTION
    verbose : int = 1

    def __post_init__(self):
        if self.startConfig is None:
            self.startConfig = self.robot.config
        if self.startObjectConfig is None and self.object is not None:
            self.startObjectConfig = self.object.getTransform()


def multi_step_plan_to_pick(world : WorldModel, robot: RobotModel,
                            object : RigidObjectModel,
                            gripper : GripperInfo,
                             graspTransform : RigidTransform,
                             graspFingerConfig : Config = None,
                             pregraspOffset : Union[float,RigidTransform] = 0.02,
                             pregraspFingerConfig : Config = None,
                             liftOffset : Union[float,RigidTransform] = 0.01) -> Tuple[trajectory.RobotTrajectory,trajectory.RobotTrajectory,trajectory.RobotTrajectory]:
    """Solves a pick and place path using the multistep module."""
    context = PickAndPlaceContext(world=world,robot=robot,gripper=gripper, object=object)
    if graspFingerConfig is None:
        graspFingerConfig = context.gripper.closedConfig
    if pregraspFingerConfig is None:
        pregraspFingerConfig = context.gripper.openConfig
    def set_gripper(context : PickAndPlaceContext, robotConfig : Config, fingerConfig : Config) -> Config:
        """Sets a finger configuration for context.gripper to the pregrasp configuration."""
        return context.gripper.setFingerConfig(robotConfig,fingerConfig)
    def straight_line_path(context : PickAndPlaceContext, a : Config, b : Config) -> Configs:
        """Creates a straight line path between two configurations."""
        return [a,b]
    def pregrasp_ik_goal(context : PickAndPlaceContext, 
                        graspTransform : RigidTransform, pregraspOffset : Union[float,RigidTransform]) -> IKObjective:
        """Computes the pregrasp configuration based on the guess configuration,
        grasp transform, pregrasp offset, and pregrasp finger configuration."""
        if isinstance(pregraspOffset, float):
            pregraspTransform = (graspTransform[0],vectorops.madd(graspTransform[1], context.gripper.primaryAxis, -pregraspOffset))
        else:
            pregraspTransform = se3.mul(graspTransform,pregraspOffset)
        if context.gripper.secondaryAxis is None:
            #assume radially symmetric, so use an axial constraint
            axisPtLocal = vectorops.madd(context.gripper.center,context.gripper.primaryAxis,0.1)
            return ik.objective(context.robot.link(context.gripper.baseLink),
                               local=[context.gripper.center,axisPtLocal],
                               world=[se3.apply(pregraspTransform,context.gripper.center),se3.apply(pregraspTransform,axisPtLocal)])
        else:
            #assume fully rotationally constrained
            return ik.objective(context.robot.link(context.gripper.baseLink),R=pregraspTransform[0],t=pregraspTransform[1])
        
    movingSubset = ik.solver(ik.fixed_objective(context.robot.link(context.gripper.baseLink))).getActiveDofs()
    space = robotcspace.RobotCSpace(robot=context.robot,collider=WorldCollider(context.world))
    space.eps=EDGE_CHECK_RESOLUTION
    space.setup()
    gripper_object_collisions = []
    if context.object is not None:
        gripper_object_collisions.append((context.object,context.robot.link(context.gripper.baseLink)))
        for finger in context.gripper.fingerLinks:
            gripper_object_collisions.append((context.object,context.robot.link(finger)))
    ignore_object_space = robotcspace.RobotCSpace(robot=context.robot,collider=WorldCollider(context.world,gripper_object_collisions))
    ignore_object_space.eps=EDGE_CHECK_RESOLUTION
    ignore_object_space.setup()
    arm_space = robotplanning.make_space(world=context.world,robot=context.robot,
                    edgeCheckResolution=EDGE_CHECK_RESOLUTION,
                    movingSubset=movingSubset)
    startConfig = multistep.constant('startConfig', context.startConfig, Config)
    openGripperConfig = multistep.node("open_gripper", set_gripper, can_fail=False)(startConfig,pregraspFingerConfig)
    openGripperPath = multistep.node("open_gripper_path", straight_line_path, can_fail=False)(startConfig, openGripperConfig)
    pregraspIK = multistep.node('pregrasp_ik', pregrasp_ik_goal, can_fail=False)(graspTransform, pregraspOffset)
    pregraspConfigIK = multistep.node("pregrasp_config_ik", multistep.IKSolverNode(context.startConfig))(pregraspIK)
    pregraspConfig = multistep.node('pregrasp_gripper',set_gripper, can_fail=False)(pregraspConfigIK,pregraspFingerConfig)
    transitPath = multistep.node("pregrasp_path", multistep.KinematicMotionPlannerNode(arm_space,PLANNER_SETTINGS))(openGripperConfig, pregraspConfig)
    graspMovedConfig = multistep.CartesianOffsetNode(context.gripper.baseLink, type='absolute')(pregraspConfig, graspTransform)
    graspClosedConfig = multistep.node('grasp_closed_config', set_gripper, can_fail=False)(graspMovedConfig, graspFingerConfig)
    graspPath = multistep.node("grasp_path", multistep.CartesianInterpolateNode(context.gripper.baseLink))(pregraspConfig, graspClosedConfig)
    liftOffset = (so3.identity(), [0,0,liftOffset]) if isinstance(liftOffset, float) else liftOffset
    liftConfig = multistep.node("lift_config", multistep.CartesianOffsetNode(context.gripper.baseLink, type='extrinsic'))(graspClosedConfig, liftOffset)
    liftPath = multistep.node("lift_path", multistep.CartesianInterpolateNode(context.gripper.baseLink))(graspClosedConfig, liftConfig)
    transitTraj = multistep.PathToTrajectoryNode()(multistep.ConcatPathsNode()(openGripperPath, transitPath))
    graspTraj = multistep.PathToTrajectoryNode()(graspPath)
    liftTraj = multistep.PathToTrajectoryNode()(liftPath)
    
    #add constraints to items, set names to aid debugging
    startConfig.add_constraint(space.isFeasible)
    openGripperConfig.set_name('openGripperConfig')
    openGripperConfig.add_constraint(space.isFeasible)
    openGripperPath.set_name('openGripperPath')
    pregraspConfigIK.set_name('pregraspConfigIK')
    pregraspConfig.set_name('pregraspConfig')
    pregraspConfig.add_constraint(space.isFeasible)
    transitPath.set_name('transitPath')
    graspClosedConfig.set_name('graspClosedConfig')
    graspClosedConfig.add_constraint(ignore_object_space.isFeasible)

    # planner = multistep.SequentialMultiStepPlanner(context)
    
    planner = multistep.IntrospectiveMultiStepPlanner(context)
    #set priors on items.  Procedural nodes' priors are already set to very cheap and likely to succeed 
    planner.set_node_prior("pregrasp_config_ik", 0.003, 0.3)  #not always successful
    planner.set_node_prior("pregrasp_path", 0.010, 0.9)  #expensive but nearly always successful
    planner.set_node_prior("grasp_path", 0.005, 0.95)     #expensive and nearly always successful
    planner.set_node_prior("lift_config", 0.005, 0.95)     #expensive and nearly always successful
    planner.set_item_prior('pregraspConfig', 0.001, 0.75)     #not always successful, may collide with the table
    planner.set_item_prior('graspClosedConfig', 0.001, 0.75)  #not always successful, may collide with the table
    
    planner.add_target(transitTraj)
    planner.add_target(graspTraj)
    planner.add_target(liftTraj)
    res = planner.solve(1000, None)
    if res is not None:
        assert isinstance(res,(tuple,list)) and len(res) == 3
        # for plan in planner.solution_plans():
        #     print("Plan found:")
        #     for item_name, value in plan.items():
        #         print("  {}: {}".format(item_name, value))
        return res
    for node,plans in planner.failed_plans().items():
        print("Node {} failed with {} plans".format(node,len(plans)))
        if node == 'pregrasp_path':
            print("   Pregrasp path failed, here are the plans:")
            for i,plan in enumerate(plans):
                vis.debug(plan['openGripperConfig'],{'color':(0,1,0,0.5)},plan['pregraspConfig'],{'color':(1,0,0,0.5)},world=world)
    return None,None,None


def multi_step_plan_to_place(world : WorldModel, robot: RobotModel,
                  grasped_object : RigidObjectModel,
                  gripper : GripperInfo,
                  placeTransform : RigidTransform,
                  placeOffset : Union[float,RigidTransform] = 0.01,
                  openGraspConfig : Config = None,
                  openGraspOffset : Union[float,RigidTransform] = 0.02,
                  supportSurfaceSymmetry : bool = True) -> Tuple[trajectory.RobotTrajectory,trajectory.RobotTrajectory,trajectory.RobotTrajectory]:
    """Plans for the robot to place a grasped object onto a surface, avoiding
    collisions with the objects in world during transit.
    
    Args:
        world (WorldModel): the world containing the robot and obstacles.
        robot (RobotModel): the moving robot to which the gripper is attached.
            Its current configuration is assumed to be the start configuration.
        grasped_object (RigidObjectModel): the object being grasped.  Its current
            transform is assumed to be the grasped transform relative to the
            gripper.
        gripper (GripperInfo): describes the gripper geometry.
        placeTransform (se3 object): the desired transform of the grasped object
            link after placing.
        placeOffset (float or RigidTransform): the amount to withdraw away
            from placeTransform at the preplace pose.  If float, the pose is
            moved upwards in the Z direction, assuming a flat surface.
        openGraspConfig (Config): the finger configuration after opening, or None
            to use the gripper's open configuration (of JUST the gripper fingers)
        openGraspOffset (float or RigidTransform): the amount to withdraw away
            from placeTransform at the postplace pose.  If float, the pose is
            moved away according to `gripper.primaryAxis`.
        supportSurfaceSymmetry (bool): if True, assumes that the object can be
            placed in any flat orientation on the support surface (e.g., a table).
    
    Returns:
        None if failed, otherwise (transfer_path,place_path,retract_path) where
        transfer_path is a path that moves the gripper to the preplace pose,
        place_path is a path that places the object down, and retract path opens
        the gripper and moves it away from the object.
    """
    context = PickAndPlaceContext(world=world,robot=robot,gripper=gripper, object=grasped_object)
    closedGraspConfig = gripper.getFingerConfig(context.startConfig)
    if openGraspConfig is None:
        openGraspConfig = gripper.openConfig
    if isinstance(openGraspOffset, float):
        openGraspOffset = (so3.identity(),vectorops.mul(gripper.primaryAxis, -openGraspOffset))
    
    #create a transfer state space
    collider = WorldCollider(world)
    cspace = robotcspace.RobotCSpaceWithObject(robot, collider)
    cspace.attachObject(grasped_object.index, gripper.baseLink)
    graspRelTransform = cspace.attachments[0][-1]  # Tgripper^-1 Tobj = T_obj^gripper
    cspace.eps = EDGE_CHECK_RESOLUTION
    cspace.setup()
    cspace_transit = robotcspace.RobotCSpace(robot, collider)
    cspace_transit.eps = EDGE_CHECK_RESOLUTION
    cspace_transit.setup()
    #create the transfer target as a transform for the gripper base link
    if isinstance(placeOffset, float):
        placeOffset = (so3.identity(), [0,0,placeOffset])
    preplaceTransform = se3.mul(placeOffset, placeTransform)
    if supportSurfaceSymmetry:
        #assume that the object can be placed in any flat orientation on the support surface
        z = vectorops.unit(placeOffset[1])
        h = 0.05
        obj_center_local = graspRelTransform[1]
        z_local = so3.apply(graspRelTransform[0],so3.apply(so3.inv(placeTransform[0]),z))  #world -> object -> gripper
        preplaceObjective = ik.objective(robot.link(gripper.baseLink),local=[obj_center_local,vectorops.madd(obj_center_local,z_local,h)],
                                         world = [preplaceTransform[1], vectorops.madd(preplaceTransform[1],z,h)])
        placeObjective = ik.objective(robot.link(gripper.baseLink),local=[obj_center_local,vectorops.madd(obj_center_local,z_local,h)],
                                         world = [placeTransform[1], vectorops.madd(placeTransform[1],z,h)])
    else:
        placeGripperTransform = se3.mul(se3.inv(graspRelTransform),placeTransform)   #T_gripper^world = (T_obj^gripper)^-1 T_obj^world 
        preplaceGripperTransform = se3.mul(se3.inv(graspRelTransform), preplaceTransform) 
        preplaceObjective = ik.objective(robot.link(gripper.baseLink),R=preplaceGripperTransform[0],t=preplaceGripperTransform[1])
        placeObjective = ik.objective(robot.link(gripper.baseLink),R=placeGripperTransform[0],t=placeGripperTransform[1])
    
    #get only DOFs that are (strict) ancestors of gripper.baseLink and that are not fixed
    movingSubset = ik.solver(ik.fixed_objective(robot.link(gripper.baseLink))).getActiveDofs()
    sspace = robotcspace.EmbeddedRobotCSpace(cspace,movingSubset,xinit=context.startConfig)
    sspace.disableInactiveCollisions()
    sspace.setup()

    startConfig = multistep.constant('startConfig', context.startConfig, Config)
    placeConfig = multistep.node('placeConfig',multistep.IKSolverNode(context.startConfig))(placeObjective)
    preplaceConfig = multistep.node('preplaceConfig', multistep.CartesianOffsetNode(gripper.baseLink, type='extrinsic'))(placeConfig, placeOffset)
    place_path = multistep.node('place_path', multistep.CartesianInterpolateNode(gripper.baseLink))(preplaceConfig, placeConfig)
    transfer_path = multistep.node('transfer_path',multistep.KinematicMotionPlannerNode(sspace,context.planner_settings))(startConfig, preplaceConfig)
    retract_config = multistep.node('retract_config', multistep.CartesianOffsetNode(gripper.baseLink, type='intrinsic'))(placeConfig, openGraspOffset)
    retract_open_config = multistep.node('open_retract',(lambda context, q, qfinger: context.gripper.setFingerConfig(q, qfinger)),can_fail=False)(retract_config, openGraspConfig)
    with multistep.set_config((context.object, placeTransform)):
        retract_path = multistep.node('retract_path', multistep.CartesianInterpolateNode(gripper.baseLink))(placeConfig, retract_open_config)
    transfer_traj = multistep.PathToTrajectoryNode()(transfer_path)
    place_traj = multistep.PathToTrajectoryNode()(place_path)
    retract_traj = multistep.PathToTrajectoryNode()(retract_path)

    startConfig.add_constraint(cspace.isFeasible)
    preplaceConfig.set_name('preplaceConfig')
    preplaceConfig.add_constraint(cspace.isFeasible)
    transfer_path.set_name('transfer_path')
    retract_open_config.set_name('retract_open_config')
    retract_open_config.add_constraint(cspace_transit.isFeasible)   #note that this should have the set_config decorator...

    # planner = multistep.SequentialMultiStepPlanner(context)
    
    planner = multistep.IntrospectiveMultiStepPlanner(context)
    #set priors on items.  Procedural nodes' priors are already set to very cheap and likely to succeed 
    planner.set_node_prior("placeConfig", 0.003, 0.75)   #not always successful
    planner.set_node_prior("transfer_path", 0.010, 0.9)  #expensive but nearly always successful
    planner.set_node_prior("place_path", 0.005, 0.95)     #expensive and nearly always successful
    planner.set_node_prior("retract_config", 0.001, 0.95)     #cheap and nearly always successful
    planner.set_item_prior('retract_path', 0.005, 0.95)     #almost always successful
    
    planner.add_target(transfer_traj)
    planner.add_target(place_traj)
    planner.add_target(retract_traj)
    res = planner.solve(1000, None)
    if res is not None:
        assert isinstance(res,(tuple,list)) and len(res) == 3
        # for plan in planner.solution_plans():
        #     print("Plan found:")
        #     for item_name, value in plan.items():
        #         print("  {}: {}".format(item_name, value))
        return res
    for node,plans in planner.failed_plans().items():
        print("Node {} failed with {} plans".format(node,len(plans)))
    return None,None,None



def plan_to_grasp(world : WorldModel, robot: RobotModel,
               gripper : GripperInfo,
               graspTransform : RigidTransform,
               graspConfig : Config = None,
               pregraspOffset : Union[float,RigidTransform] = 0.02,
               pregraspConfig : Config = None,
               settings : dict = PLANNER_SETTINGS, maxIters : int = MAX_PLANNER_ITERS, maxTime : float = MAX_PLANNER_TIME,
               edgeCheckResolution : float = EDGE_CHECK_RESOLUTION,
               verbose : int = 1) -> Tuple[Optional[Configs],Optional[Configs]]:
    """Plans for the robot to move to a grasp configuration, avoiding collisions
    with the objects in world during the pregrasp phase.

    Args:
        world (WorldModel): the world containing the robot and obstacles.
        robot (RobotModel): the moving robot to which the gripper is attached.
            Its current configuration is assumed to be the start configuration.
        gripper (GripperInfo): describes the gripper geometry.
        graspTransform (se3 object): the desired transform of the gripper base
            link after grasping.
        graspConfig (Config): the finger configuration after grasping (of JUST
            the gripper fingers.) 
        pregraspOffset (float or RigidTransform): the amount to withdraw away
            from graspTransform at the pregrasp pose.  If float, the pose is
            moved away according to `gripper.primaryAxis`.
        pregraspConfig (Config): the pregrasp finger configuration, or None to
            use the gripper's open configuration (of JUST the gripper fingers)

    Returns:
        (transit,grasp) or None: None if failed.  If successful, a pair consisting
            of a transit path that is collision free, and a grasp path that may
            collide.
    """
    startConfig = robot.config
    #first open the gripper
    if graspConfig is None:
        graspConfig = gripper.closedConfig
    if pregraspConfig is None:
        pregraspConfig = gripper.openConfig
    openConfig = gripper.setFingerConfig(startConfig,pregraspConfig)
    prefix_path = [startConfig,openConfig]

    pregraspTransform = (graspTransform[0],vectorops.madd(graspTransform[1],gripper.primaryAxis,-pregraspOffset))
    if gripper.secondaryAxis is None:
        #assume radially symmetric, so use an axial constraint
        axisPtLocal = vectorops.madd(gripper.center,gripper.primaryAxis,0.1)
        pregrasp_ik = ik.objective(robot.link(gripper.baseLink),
                               local=[gripper.center,axisPtLocal],
                               world=[se3.apply(pregraspTransform,gripper.center),se3.apply(pregraspTransform,axisPtLocal)])
        grasp_ik = ik.objective(robot.link(gripper.baseLink),
                                local=[gripper.center,axisPtLocal],
                                world=[se3.apply(graspTransform,gripper.center),se3.apply(graspTransform,axisPtLocal)])
    else:
        #assume fully rotationally constrained
        pregrasp_ik = ik.objective(robot.link(gripper.baseLink),R=pregraspTransform[0],t=pregraspTransform[1])
        grasp_ik = ik.objective(robot.link(gripper.baseLink),R=graspTransform[0],t=graspTransform[1])
    t0 = time.time()
    #get only DOFs that are (strict) ancestors of gripper.baseLink and that are not fixed
    movingSubset = ik.solver(ik.fixed_objective(robot.link(gripper.baseLink))).getActiveDofs()

    t0 = time.time()
    robot.setConfig(openConfig)
    plan = robotplanning.plan_to_cartesian_objective(world,robot,pregrasp_ik,
                                        movingSubset=movingSubset,
                                        edgeCheckResolution=edgeCheckResolution,
                                        **settings)
    if verbose >= 1:
        print("Planner creation time",time.time()-t0)
    transit_path = robotplanning.run_plan(plan,maxIters,maxTime,verbose=verbose)
    #to be nice to the C++ module, do this to free up memory
    plan.space.close()
    plan.close()
    if transit_path is None:
        return None,None
    #close the fingers and move in to the grasp
    #TODO: use cartesian interpolation to do this
    if not ik.solve_nearby(grasp_ik, 0.5):
        print("Could not solve grasp IK problem")
        return transit_path,None
    finalConfig = gripper.setFingerConfig(robot.config,graspConfig)
    #assume this is close enough to avoid requiring a discretized motion?
    #TODO: collision checking the grasp path
    return (prefix_path[0:1] + transit_path, [transit_path[-1],finalConfig])


def plan_to_pick(world : WorldModel, robot: RobotModel,
               gripper : GripperInfo,
               graspTransform : RigidTransform,
               graspConfig : Config = None,
               pregraspOffset : Union[float,RigidTransform] = 0.02,
               pregraspConfig : Config = None,
               liftOffset : Union[float,RigidTransform] = 0.01,
               settings : dict = PLANNER_SETTINGS, maxIters : int = MAX_PLANNER_ITERS, maxTime : float = MAX_PLANNER_TIME,
               edgeCheckResolution : float = EDGE_CHECK_RESOLUTION,
               verbose : int = 1) -> Tuple[Optional[Configs],Optional[Configs],Optional[Configs]]:
    """Plans for the robot to grasp an object and lift it, avoiding collisions
    with the objects in world during the pregrasp phase.

    Args:
        world (WorldModel): the world containing the robot and obstacles.
        robot (RobotModel): the moving robot to which the gripper is attached.
            Its current configuration is assumed to be the start configuration.
        gripper (GripperInfo): describes the gripper geometry.
        graspTransform (se3 object): the desired transform of the gripper base
            link after grasping.
        graspConfig (Config): the finger configuration after grasping (of JUST
            the gripper fingers.) 
        pregraspOffset (float or RigidTransform): the amount to withdraw away
            from graspTransform at the pregrasp pose.  If float, the pose is
            moved away according to `gripper.primaryAxis`.
        pregraspConfig (Config): the pregrasp finger configuration, or None to
            use the gripper's open configuration (of JUST the gripper fingers)

    Returns:
        (transit,grasp,lift) or None: None if failed.  If successful, a tuple
            consisting of a transit path that is collision free, a grasp path
            that may collide, and a lift path that may collide.
    """
    transit,grasp = plan_to_grasp(world,robot,gripper,graspTransform,graspConfig,
                             pregraspOffset,pregraspConfig,settings,maxIters,maxTime,edgeCheckResolution,verbose)
    if transit is None or grasp is None:
        return transit,grasp,None
    robot.setConfig(grasp[-1])
    graspTransform = robot.link(gripper.baseLink).getTransform()
    liftTransform = (graspTransform[0],vectorops.madd(graspTransform[1],[0,0,1],liftOffset))
    liftPath = cartesian_trajectory.cartesian_move_to(robot,
                                                      ik.objective(robot.link(gripper.baseLink),R=liftTransform[0],t=liftTransform[1]))
    if liftPath is None:
        return transit,grasp,None
    #TODO: collision checking the lift path
    return transit,grasp,liftPath.milestones


def plan_to_place(world : WorldModel, robot: RobotModel,
                  grasped_object : RigidObjectModel,
                  gripper : GripperInfo,
                  placeTransform : RigidTransform,
                  placeOffset : Union[float,RigidTransform] = 0.01,
                  openGraspConfig : Config = None,
                  openGraspOffset : Union[float,RigidTransform] = 0.02) -> Tuple[Optional[Configs],Optional[Configs],Optional[Configs]]:
    """Plans for the robot to place a grasped object onto a surface, avoiding
    collisions with the objects in world during transit.
    
    Args:
        world (WorldModel): the world containing the robot and obstacles.
        robot (RobotModel): the moving robot to which the gripper is attached.
            Its current configuration is assumed to be the start configuration.
        grasped_object (RigidObjectModel): the object being grasped.  Its current
            transform is assumed to be the grasped transform relative to the
            gripper.
        gripper (GripperInfo): describes the gripper geometry.
        placeTransform (se3 object): the desired transform of the grasped object
            link after placing.
        placeOffset (float or RigidTransform): the amount to withdraw away
            from placeTransform at the preplace pose.  If float, the pose is
            moved away according to `gripper.primaryAxis`.
        openGraspConfig (Config): the finger configuration after opening, or None
            to use the gripper's open configuration (of JUST the gripper fingers)
        openGraspOffset (float or RigidTransform): the amount to withdraw away
            from placeTransform at the postplace pose.  If float, the pose is
            moved away according to `gripper.primaryAxis`.
    
    Returns:
        None if failed, otherwise (transfer_path,place_path,retract_path) where
        transfer_path is a path that moves the gripper to the preplace pose,
        place_path is a path that places the object down, and retract path opens
        the gripper and moves it away from the object.
    """
    startConfig = robot.config
    #create a transfer state space
    collider = WorldCollider(world)
    cspace = robotcspace.RobotCSpaceWithObject(robot, collider)
    cspace.attachObject(grasped_object.index, gripper.baseLink)
    graspRelTransform = cspace.attachments[0][-1]  # Tgripper^-1 Tobj
    cspace.eps = EDGE_CHECK_RESOLUTION
    #create the transfer target as a transform for the gripper base link
    placeGripperTransform = se3.mul(placeTransform, se3.inv(graspRelTransform)) 
    preplaceGripperTransform = (placeGripperTransform[0], vectorops.madd(placeGripperTransform[1], [0,0,1], placeOffset))
    goalset = robotcspace.ClosedLoopRobotCSpace(robot,[ik.objective(robot.link(gripper.baseLink),R=preplaceGripperTransform[0],t=preplaceGripperTransform[1])])
    goalset.tol = 1e-3  #tolerance for the goal set

    cspace.setup()
    if not cspace.isFeasible(startConfig):
        print("Start configuration is not feasible, fails",cspace.cspace.feasibilityFailures(startConfig))
        return None,None,None

    #get only DOFs that are (strict) ancestors of gripper.baseLink and that are not fixed
    movingSubset = ik.solver(ik.fixed_objective(robot.link(gripper.baseLink))).getActiveDofs()
    sspace = robotcspace.EmbeddedRobotCSpace(cspace,movingSubset,xinit=startConfig)
    sspace.disableInactiveCollisions()
    sspace.setup()
    #run the planner
    plan = robotplanning.EmbeddedKinematicPlanner(sspace,startConfig,**PLANNER_SETTINGS)
    plan.setEndpoints(startConfig, (goalset.feasible, goalset.sample))
    transfer_path = robotplanning.run_plan(plan, maxIters=MAX_PLANNER_ITERS, maxTime=MAX_PLANNER_TIME, verbose=1)
    if transfer_path is None:
        print("Failed to find a transfer path")
        return None,None,None
    #set the object down via placeGripperTransform
    robot.setConfig(transfer_path[-1])
    place_path = cartesian_trajectory.cartesian_move_to(robot,ik.objective(robot.link(gripper.baseLink),R=placeGripperTransform[0],t=placeGripperTransform[1]))
    if place_path is None:
        print("Failed to find a place path")
        return transfer_path,None,None
    #open the gripper and retract
    if openGraspConfig is None:
        openGraspConfig = gripper.openConfig
    closedGraspConfig = gripper.getFingerConfig(place_path.milestones[-1])
    openGripperTransform = (placeGripperTransform[0], vectorops.madd(placeGripperTransform[1], gripper.primaryAxis, openGraspOffset))
    retract_path = cartesian_trajectory.cartesian_move_to(robot,
        ik.objective(robot.link(gripper.baseLink),R=openGripperTransform[0],t=openGripperTransform[1]))
    if retract_path is None:
        print("Failed to find a retract path")
        return transfer_path, place_path.milestones,None
    #interpolate the grasp configuration along the cartesian path
    for i in range(len(retract_path.milestones)):
        t = retract_path.times[i] / retract_path.times[-1]
        graspConfig = vectorops.interpolate(closedGraspConfig, openGraspConfig, t)
        retract_path.milestones[i] = gripper.setFingerConfig(retract_path.milestones[i], graspConfig)
    return transfer_path, place_path.milestones, retract_path.milestones


def main(robotinfofn: str, worldfn : str):
    robotinfo = RobotInfo.load(robotinfofn)
    if len(robotinfo.grippers) == 0:
        print("No grippers found in robot info",robotinfofn)
        exit(0)
    
    #load the robot / world file
    world = WorldModel()
    res = world.readFile(worldfn)
    if not res:
        print("Unable to read file",worldfn)
        exit(0)

    robot = world.robot(0)
    assert robotinfo.klamptModel().numLinks() == robot.numLinks(), \
        "Robot info {} does not match robot {}".format(robotinfofn,robot.getName())
    if world.numRigidObjects() == 0:
        print("No rigid objects found in world",worldfn)
        exit(0)
    
    #add the world elements individually to the visualization
    vis.add("robot",robot)
    for i in range(1,world.numRobots()):
        vis.add("robot"+str(i),world.robot(i))
    for i in range(world.numRigidObjects()):
        vis.add("rigidObject"+str(i),world.rigidObject(i))
    for i in range(world.numTerrains()):
        vis.add("terrain"+str(i),world.terrain(i))


    if SIMPLIFY_TYPE is not None:
        #this line replaces the robot's normal geometry with bounding boxes.
        #it makes planning faster but sacrifices accuracy.
        print("#########################################")
        print("Simplifying world/robot to",SIMPLIFY_TYPE)
        print("#########################################")
        from klampt.model import geometry
        geometry.convert(world,SIMPLIFY_TYPE)

        #if you want to just see the robot in a pop up window...
        if DEBUG_SIMPLIFY:
            print("#########################################")
            print("Showing the simplified world")
            print("#########################################")
            vis.setWindowTitle("Simplified world")
            vis.dialog()

    vis_robot = robotinfo.klamptModel()
    gripper_info = list(robotinfo.grippers.values())[0]
    Tgripper = robot.link(gripper_info.baseLink).getTransform()
    Tgripper = world.rigidObject(0).getTransform()   #move to the rigid object frame
    Tgripper = (Tgripper[0],vectorops.add(Tgripper[1],[-0.2,0,0]))
    vis.add("gripper_base",Tgripper,hide_label=True)
    gripper_info.addToVis(vis_robot,animate=False,base_xform = Tgripper)
    gripper_editor = vis.edit("gripper_base")
    obj_ghost = world.rigidObject(0).geometry().copy()
    Tobject = world.rigidObject(0).getTransform()
    obj_ghost.setCurrentTransform(Tobject[0],vectorops.add(Tobject[1],[0,-0.3,0]))
    vis.add("obj_ghost",obj_ghost,color=(1,0,0,0.5),hide_label=True)
    vis.add("obj_placement",obj_ghost.getCurrentTransform(),hide_label=True)
    obj_ghost_editor = vis.edit("obj_placement")

    vis.setWindowTitle("Choose grasp pose and target object pose")
    vis.show()
    t0 = time.time()
    while vis.shown():
        t = time.time() - t0
        #sawtooth wave for animation
        u = (t % 3.0) / 3.0
        u = 1.0 - abs(u - 0.5) * 2.0
        vis.lock()
        vis_robot.config = gripper_info.setFingerConfig(vis_robot.getConfig(),gripper_info.partwayOpenConfig(u))
        vis.unlock()
        Tgripper = gripper_editor.get()
        gripper_info.addToVis(robotinfo.klamptModel(),animate=False,base_xform = Tgripper)
        Tplacement = obj_ghost_editor.get()
        obj_ghost.setCurrentTransform(*Tplacement)
        time.sleep(0.03)
    
    gripper_info.removeFromVis()
    vis.remove("obj_placement")
    vis.remove("gripper_base")

    transit_traj,grasp_traj,lift_traj = multi_step_plan_to_pick(world, robot, world.rigidObject(0), gripper_info, Tgripper)
    if transit_traj is None or grasp_traj is None or lift_traj is None:
        print("Failed to find a pick path")
        return
    whole_traj = transit_traj.concat(grasp_traj,relative=True).concat(lift_traj,relative=True)
    # transit,grasp,lift = plan_to_pick(world, robot, gripper_info, Tgripper)
    # if transit is None or grasp is None or lift is None:
    #     print("Failed to find a pick path")
    #     return
    # whole_path = transit + grasp[1:] + lift[1:]
    # whole_traj = trajectory.path_to_trajectory(trajectory.RobotTrajectory(robot,None,whole_path),smoothing=None)
    vis.add("pick_path",whole_traj,color=(0,1,0,0.5),hide_label=True)
    vis.animate("robot",whole_traj)
    vis.setWindowTitle("Pick path")
    vis.show()
    while vis.shown():
        time.sleep(0.03)
    vis.remove("pick_path")
    
    robot.setConfig(grasp_traj.milestones[-1])
    transfer,place,retract = multi_step_plan_to_place(world, robot, world.rigidObject(0), gripper_info, Tplacement)
    # transfer, place, retract = plan_to_place(world, robot, world.rigidObject(0), gripper_info, Tplacement)
    # if transfer is None or place is None or retract is None:
    #     print("Failed to find a place path")
    #     return
    # whole_path = transfer + place[1:] + retract[1:]
    # whole_traj = trajectory.path_to_trajectory(trajectory.RobotTrajectory(robot,None,whole_path),smoothing=None)
    if transfer is None or place is None or retract is None:
        print("Failed to find a place path")
        return
    whole_traj = transfer.concat(place,relative=True).concat(retract,relative=True)
    vis.add("place_path",whole_traj,color=(0,0,1,0.5),hide_label=True)
    vis.animate("robot",whole_traj)
    vis.setWindowTitle("Place path")
    vis.show()
    while vis.shown():
        time.sleep(0.03)
    vis.remove("place_path")

    vis.kill()

if __name__ == '__main__':
    robotinfo_fn = "../../../robotinfo/tx90pr2_sim.json"
    world_fn = "../../../data/manipulation_worlds/tx90cuptable.xml"
    if len(sys.argv) > 2:
        robotinfo_fn = sys.argv[1]
        world_fn = sys.argv[2]
    main(robotinfo_fn, world_fn)
    