from klampt.manip.grasp_space import GraspParameterSpace
from klampt.manip.primitive_grasp_sampler import PrimitiveGraspSampler
from klampt.model.gripperinfo import GripperInfo
from klampt.model.create import box, bbox, sphere
from klampt.math import se3,so3
from klampt import WorldModel
from klampt import vis
import numpy as np

if __name__ == '__main__':
    world = WorldModel()
    #world.readFile('../../../data/gripper_robots/robotiq_epick.rob')
    # gripper = GripperInfo.load('../../../robotinfo/gripperinfo/robotiq_epick.json')
    world.readFile('../../../data/gripper_robots/robotiq_85.rob')
    gripper = GripperInfo.load('../../../robotinfo/gripperinfo/robotiq_85.json')
    sampler = PrimitiveGraspSampler(gripper)
    #sampler = PrimitiveGraspSampler(gripper,centrality_penalty=1.0)
    robot = world.robot(0)
    obj = world.makeRigidObject('target')
    #5cm sphere mesh treated as bounding box
    # obj.geometry().loadFile('../../../data/objects/sphere.off')
    # obj.geometry().transform([0.025,0,0,0,0.025,0,0,0,0.025],[0,0,0])
    # obj.geometry().set(bbox([0,0,0],[0.04,0.07,0.12],type='GeometricPrimitive'))
    obj.geometry().set(sphere(0.06,[0,0,0],type='GeometricPrimitive'))
    obj.setTransform(so3.identity(),[0.6,0.1,-0.3])

    sampler.init(world, obj)
    print("Best score",sampler.score())
    vis.add("origin",se3.identity(),hide_label=True,fancy=True,length=1.0)
    vis.add("world",world)
    sampler.visDebug()
    if sampler.options is not None:
        for i,(g,s) in enumerate(sampler.options):
            T = g.asFixedGrasp().ikConstraint.getTransform()
            robot.config = gripper.setFingerConfig(robot.config, g.fingerConfig)
            gripper.addToVis(robot, base_xform = T, animate=False, prefix=f"grasp_{i}")
            break
    vis.loop()
