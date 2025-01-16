from klampt.plan import robotcspace
from klampt.plan import cspace
from klampt.plan import robotplanning
from klampt.math import se3
from klampt import vis 
from klampt.io import resource
from klampt.model import ik
from klampt.model import trajectory
from klampt.model.collide import WorldCollider
from klampt import *
from typing import List,Tuple,Union,Any
from klampt.model.typing import Config
import time
import sys

#Default motion planner settings
#ACTIVE_DOFS = 'all'   #this will plan for all DOFs
ACTIVE_DOFS = 'auto'  #this will restrict planning to only the DOFs that move
#set up a settings dictionary here.  This is a random-restart + shortcutting
#SBL planner.
PLANNER_SETTINGS = { 'type':"sbl", 'perturbationRadius':0.5, 'bidirectional':1, 'shortcut':1, 'restart':1, 'restartTermCond':"{foundSolution:1,maxIters:1000}" }
IS_PLANNER_OPTIMIZING = True
#You might edit the values 500 and 10 to play with how many iterations /time 
#to give the planner.
MAX_PLANNER_ITERS = 500
MAX_PLANNER_TIME = 10.0
EDGE_CHECK_RESOLUTION = 0.01

#Setup settings: feel free to edit these to see how the results change
#Simplify geometry or not?
#SIMPLIFY_TYPE = None
#SIMPLIFY_TYPE = 'aabb'
SIMPLIFY_TYPE = 'ConvexHull'
DEBUG_SIMPLIFY = 0
#Create CSpace manually
MANUAL_SPACE_CREATION = 0
#Impose a closed-loop constraint on the problem
CLOSED_LOOP_TEST = 0
#Plan to cartesian goals rather than configurations exactly
PLAN_CARTESIAN_TEST = 0
#Show the smoothed path rather than the raw one
SHOW_SMOOTHED_PATH = 1

def run_planner_default(plan : cspace.MotionPlan,
                        maxIters : int = MAX_PLANNER_ITERS,
                        maxTime : float = MAX_PLANNER_TIME,
                        endpoints : Tuple = None,
                        verbose : int=1)  -> Union[None,List[Config]]:
    """A default runner for a planner, works generally in a sane manner for 
    most defaults and allows debugging by setting verbose >= 1.

    Args:
        plan (MotionPlan): a MotionPlan object at least partially configured.
        maxIters (int): the maximum number of iterations to run.
        maxTime (float): the maximum number of seconds to run.
        endpoints (None or pair): the endpoints of the plan, either Configs
            or goal specifications. If None uses the endpoints configured in
            plan.
        verbose (int): whether to print information about the planning.

    Returns:
        path or None: if successful,returns a path solving the terminal 
        conditions specified in the plan.
    """
    if endpoints is not None:
        if len(endpoints) != 2:
            raise ValueError("Need exactly two endpoints")
        try:
            plan.setEndpoints(*endpoints)
        except RuntimeError:
            #must be invalid configuration
            if verbose:
                if isinstance(endpoints[0],(list,tuple)) and isinstance(endpoints[0][0],(float,int)):
                    print("Start configuration fails:",plan.space.feasibilityFailures(endpoints[0]))
                if isinstance(endpoints[1],(list,tuple)) and isinstance(endpoints[1][0],(float,int)):
                    print("Goal configuration fails:",plan.space.feasibilityFailures(endpoints[1]))
            return None
    #this is helpful to slightly speed up collision queries
    plan.space.cspace.enableAdaptiveQueries(True)

    t0 = time.time()

    #begin planning
    numIters = 0
    for round in range(maxIters//10):
        print("Round",round,"of planmore(10) iterations")
        plan.planMore(10)
        numIters += 10

        if not IS_PLANNER_OPTIMIZING:  #break on first path found
            path = plan.getPath()
            if path is not None and len(path)>0:
                break
        if time.time()-t0 > maxTime:
            break
    if verbose >= 1:
        print("Planning time {}s over {} iterations".format(time.time()-t0,numIters))

    #this code just gives some debugging information. it may get expensive
    if verbose >= 2:
        V,E = plan.getRoadmap()
        print(len(V),"feasible milestones sampled,",len(E),"edges connected")
    
    if verbose >= 2:
        print("Planner stats:")
        print(plan.getStats())

    path = plan.getPath()
    if path is None or len(path)==0:
        if verbose >= 1:
            print("Failed to plan feasible path")
            if verbose < 2:
                print("Planner stats:")
                print(plan.getStats())
            #debug some sampled configurations
            if verbose >= 2:
                print("Some sampled configurations:")
                print(V[0:min(10,len(V))])
        return None

    return path

def plan_simple(config_start : Config, config_goal : Config, world : WorldModel, robot: RobotModel,
               settings : dict = PLANNER_SETTINGS, maxIters : int = MAX_PLANNER_ITERS, maxTime : float = MAX_PLANNER_TIME,
               edgeCheckResolution : float = EDGE_CHECK_RESOLUTION,
               space : Union[cspace.CSpace,str]='auto',
               verbose : int = 1) -> Union[None,List[Config]]:
    """Plans for the robot to move from config_start to config_goal,
    avoiding collisions with the objects in world.

    Returns:
        path or None: a sequence of milestones connecting config_start and config_goal,
        if successfully solved.
    """
    t0 = time.time()
    if space == 'auto':
        #this is the simple way of doing it
        t0 = time.time()
        robot.setConfig(config_start)    
        plan = robotplanning.plan_to_config(world,robot,config_goal,
                                          movingSubset=ACTIVE_DOFS,
                                          **settings)
        if verbose >= 1:
            print("Planner creation time",time.time()-t0)
        res = run_planner_default(plan,maxIters,maxTime,verbose=verbose)
        #to be nice to the C++ module, do this to free up memory
        plan.space.close()
        plan.close()
        return res
    else:
        if not isinstance(space,cspace.CSpace):
            raise ValueError("Invalid space {} specified, must be subclass of CSpace".format(space.__class__.__name__))
        #Manual construction of planner
        t0 = time.time()
        plan = cspace.MotionPlan(space, **settings)
        if verbose >= 1:
            print("Planner creation time",time.time()-t0)
        #TODO: if a subset of DOFs are used, need to project start to subset
        res = run_planner_default(plan,maxIters,maxTime,endpoints=(config_start,config_goal),verbose=verbose)
        #to be nice to the C++ module, do this to free up memory
        plan.close()
        return res


def plan_cartesian(config_start : Config, ik_goal : Union[IKObjective,List[IKObjective]],
               world : WorldModel, robot: RobotModel,
               settings : dict = PLANNER_SETTINGS, maxIters : int = MAX_PLANNER_ITERS, maxTime : float = MAX_PLANNER_TIME,
               edgeCheckResolution : float = EDGE_CHECK_RESOLUTION,
               space : Union[cspace.CSpace,str]='auto',
               verbose : int = 1) -> Union[None,List[Config]]:
    """Plans for the robot to move from config_start to ik_goal,
    avoiding collisions with the objects in world.

    Returns:
        path or None: a sequence of milestones connecting config_start to a
        configuration satisfying the constraint(s) ik_goal, or None if failed.
    """
    t0 = time.time()
    if space == 'auto':
        #this is the simple way of doing it
        t0 = time.time()
        robot.setConfig(config_start)    
        plan = robotplanning.plan_to_cartesian_objective(world,robot,ik_goal,
                                          movingSubset=ACTIVE_DOFS,
                                          **settings)
        if verbose >= 1:
            print("Planner creation time",time.time()-t0)
        res = run_planner_default(plan,maxIters,maxTime,verbose=verbose)
        #to be nice to the C++ module, do this to free up memory
        plan.space.close()
        plan.close()
        return res
    else:
        if not isinstance(space,cspace.CSpace):
            raise ValueError("Invalid space {} specified, must be subclass of CSpace".format(space.__class__.__name__))
        #Manual construction of planner
        t0 = time.time()
        plan = cspace.MotionPlan(space, **settings)
        if verbose >= 1:
            print("Planner creation time",time.time()-t0)
        ikConstraints = ik_goal if isinstance(ik_goal,(list,tuple)) else [ik_goal]
        goalset = robotcspace.ClosedLoopRobotCSpace(robot,ikConstraints,None)
        #TODO: if a subset of DOFs are used, need to project start to subset
        #TODO: if a subset of DOFs are used, need to project goal test back to full robot and sample to subset
        goal = [(lambda x:goalset.feasible(x)),(lambda : goalset.sample())]
        res = run_planner_default(plan,maxIters,maxTime,endpoints=(config_start,goal),verbose=verbose)
        #to be nice to the C++ module, do this to free up memory
        plan.close()
        return res


def plan_waypoints(configs : List[Config], world : WorldModel, robot: RobotModel,
                   settings : dict = PLANNER_SETTINGS, maxIters : int = MAX_PLANNER_ITERS, maxTime : float = MAX_PLANNER_TIME,
                   edgeCheckResolution : float = EDGE_CHECK_RESOLUTION,
                   space : Union[cspace.CSpace,str]='auto',
                   verbose : int = 1) -> Tuple[int,List[Config]]:
    """Plans for the robot to move sequentially between the configurations in 
    configs, avoiding collisions with the objects in world.

    Returns:
        tuple: (numSolved,path) where numSolved is the number of segments 
        solved and path is the path up to configs[numSolved+1].
    """
    
    wholepath = [configs[0]]
    tstart = time.time()
    for i in range(len(configs)-1):
        #run segment plan
        if verbose >= 1:
            print("Planning for segment",i,"...")

        path = plan_simple(configs[i],configs[i+1],world,robot,settings,maxIters,maxTime,edgeCheckResolution,space,verbose)
        if path is None:
            return (i-1,wholepath)
        
        wholepath += path[1:]

    if verbose >= 2 and space != 'auto':
        print("CSpace stats:")
        print(plan.space.getStats())

    if verbose >= 1 and len(configs) > 2:
        print("Total planning time for",len(configs)-1,"segments:",time.time()-tstart)
    return len(configs)-1,wholepath


def plan_cartesian_waypoints(config0 : Config, ikSequence : List[IKObjective],
                   world : WorldModel, robot: RobotModel,
                   settings : dict = PLANNER_SETTINGS, maxIters : int = MAX_PLANNER_ITERS, maxTime : float = MAX_PLANNER_TIME,
                   edgeCheckResolution : float = EDGE_CHECK_RESOLUTION,
                   space : Union[cspace.CSpace,str]='auto',
                   verbose : int = 1) -> Tuple[int,List[Config]]:
    """Plans for the robot to move sequentially between the Cartesian goals in
    ikSequence, avoiding collisions with the objects in world.

    Note: not a probabilistically complete / asymptotically method -- each goal
    in the sequence is solved greedily and the configuration reached is used as
    the start for the next goal.

    Returns:
        tuple: (numSolved,path) where numSolved is the number of segments 
        solved and path is the path up to configs[numSolved+1].
    """
    wholepath = [config0]
    tstart = time.time()
    for i in range(len(ikSequence)):
        #run segment plan
        if verbose >= 1:
            print("Planning for segment",i,"...")

        path = plan_cartesian(wholepath[-1],ikSequence[i],world,robot,settings,maxIters,maxTime,edgeCheckResolution,space,verbose)        
        if path is None:
            return (i,wholepath)

        wholepath += path[1:]

    if verbose >= 2 and space != 'auto':
        print("CSpace stats:")
        print(space.getStats())

    if verbose >= 1 and len(ikSequence) > 1:
        print("Total planning time for",len(ikSequence),"segments:",time.time()-tstart)
    return len(ikSequence),wholepath



def main(worldfn,inWaypointsFn,outputPathFn):
    #load the robot / world file
    world = WorldModel()
    res = world.readFile(worldfn)
    if not res:
        print("Unable to read file",worldfn)
        exit(0)

    robot = world.robot(0)

    #add the world elements individually to the visualization
    vis.add("robot",robot)
    for i in range(1,world.numRobots()):
        vis.add("robot"+str(i),world.robot(i))
    for i in range(world.numRigidObjects()):
        vis.add("rigidObject"+str(i),world.rigidObject(i))
    for i in range(world.numTerrains()):
        vis.add("terrain"+str(i),world.terrain(i))


    def simplify(world):
        """Replaces a world's geometry with simplified bounding
        boxes or convex hulls."""
        objects = []
        for j in range(world.numRobots()):
            robot = world.robot(j)
            for i in range(robot.numLinks()):
                objects.append(robot.link(i))
        for j in range(world.numRigidObjects()):
            obj = world.rigidObject(j)
            objects.append(obj)
        for j in range(world.numTerrains()):
            terr = world.terrain(j)
            objects.append(terr)
        for obj in objects:
            geom = obj.geometry()
            if geom.empty(): continue
            geom.setCurrentTransform(*se3.identity())
            if SIMPLIFY_TYPE == 'aabb':
                if geom.numElements() > 12:
                    BB = geom.getBBTight()
                    print(BB[0],BB[1])
                    BBgeom = GeometricPrimitive()
                    BBgeom.setAABB(BB[0],BB[1])
                    geom.setGeometricPrimitive(BBgeom)
            else:
                geom2 = geom.convert(SIMPLIFY_TYPE)
                obj.geometry().set(geom2)


    if SIMPLIFY_TYPE is not None:
        #this line replaces the robot's normal geometry with bounding boxes.
        #it makes planning faster but sacrifices accuracy.
        print("#########################################")
        print("Simplifying world/robot to bounding boxes")
        print("#########################################")
        simplify(world)

        #if you want to just see the robot in a pop up window...
        if DEBUG_SIMPLIFY:
            print("#########################################")
            print("Showing the simplified world")
            print("#########################################")
            vis.setWindowTitle("Simplified world")
            vis.dialog()

    #Automatic construction of space
    if not CLOSED_LOOP_TEST:
        if not MANUAL_SPACE_CREATION:
            space = robotplanning.makeSpace(world=world,robot=robot,
                                            edgeCheckResolution=EDGE_CHECK_RESOLUTION,
                                            movingSubset=ACTIVE_DOFS)
        else:
            #Manual construction of space -- could configure which collision tests to run using collider
            collider = WorldCollider(world)
            space = robotcspace.RobotCSpace(robot,collider)
            space.eps = EDGE_CHECK_RESOLUTION
            space.setup()
    else:
        #TESTING: closed loop robot cspace
        collider = WorldCollider(world)
        obj = ik.objective(robot.link(robot.numLinks()-1),local=[0,0,0],world=[0.5,0,0.5])
        vis.add("IK goal",obj)
        vis.dialog()
        space = robotcspace.ClosedLoopRobotCSpace(robot,obj,collider)
        space.eps = EDGE_CHECK_RESOLUTION
        space.setup()

    #Generate some waypoint configurations using the resource editor
    print("#########################################")
    print("Editing the waypoints in resources/{}/{}".format(robot.getName(),inWaypointsFn))
    print("#########################################")
    resource.setDirectory("resources/"+robot.getName())
    configs = resource.get(inWaypointsFn,"Configs",default=[],world=world,doedit=False)
    cindex = 0
    while True:
        if cindex < len(configs):
            robot.setConfig(configs[cindex])
        (save,q) = resource.edit("plan config "+str(cindex+1),robot.getConfig(),"Config",world=world,description="Press OK to add waypoint, cancel to stop")
        if save:
            if False and not space.feasible(q):
                print("Configuration is infeasible. Failures:")
                print(" ",space.cspace.feasibilityFailures(q))
                print("Please pick another")
            else:
                if cindex >= len(configs):
                    configs.append(q)
                else:
                    configs[cindex] = q
                cindex += 1
        else:
            break
    if cindex==0:
        vis.kill()
        exit(0)

    configs = configs[:cindex]
    resource.set(inWaypointsFn,configs)

    #do the planning
    if not CLOSED_LOOP_TEST and not PLAN_CARTESIAN_TEST:
        #normal planning between waypoints
        numSolved,wholepath = plan_waypoints(configs,world,robot)
    elif PLAN_CARTESIAN_TEST:
        #planning between cartesian goals
        ikTargets = []
        for q in configs[1:]:
            robot.setConfig(q)
            ikTargets.append(ik.fixed_objective(robot.link(robot.numLinks()-1),local=[0,0,0]))
        numSolved,wholepath = plan_cartesian_waypoints(configs[0],ikTargets,world,robot)
    elif CLOSED_LOOP_TEST:
        #obeying closed-loop constraints
        #need to project configurations onto the closed-loop manifold
        for i,q in enumerate(configs):
            configs[i] = space.solveConstraints(q)
        resource.edit("IK solved configs",configs,"Configs",description="These configurations try to solve the IK constraint",world=world)
        numSolved,wholepath = plan_waypoints(configs,world,robot,space=space)
        #need to generate piecewise linear trajectory in robot's joint space
        if len(wholepath)>1:
            wholepath = space.discretize(wholepath)

    #This code generates a PRM with no specific endpoints
    #plan = cspace.MotionPlan(space, "prm", knn=10)
    #print "Planning..."
    #plan.planMore(500)
    #V,E = plan.getRoadmap()
    #print(len(V),"feasible milestones sampled,",len(E),"edges connected")

    if len(wholepath)>1:
        #draw the path as a RobotTrajectory (you could just animate wholepath, but for robots with non-standard joints
        #the results will often look odd).  Animate with 5-second duration
        times = [i*5.0/(len(wholepath)-1) for i in range(len(wholepath))]
        traj = trajectory.RobotTrajectory(robot,times=times,milestones=wholepath)
        if SHOW_SMOOTHED_PATH:
            traj = trajectory.path_to_trajectory(traj)

        #if you want to save the path to disk...
        if outputPathFn is not None:
            print("Saving to",outputPathFn)
            traj.save(outputPathFn)

        #show the path in the visualizer, repeating for 60 seconds
        vis.animate("robot",traj)
        vis.spin(60)
    else:
        print("Failed to generate a plan")

    vis.kill()

if __name__ == '__main__':
    fn = "../../data/robots/jaco.rob"
    if len(sys.argv) > 1:
        fn = sys.argv[1]
    main(fn,'planningtest.configs','test.path')
    