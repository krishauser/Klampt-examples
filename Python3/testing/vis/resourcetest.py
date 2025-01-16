from klampt import *
from klampt.model.trajectory import Trajectory,SE3Trajectory,RobotTrajectory
from klampt.math import vectorops,so3,se3
from klampt.io import resource


def make_world():
    w = WorldModel()
    w.readFile("../../../data/tx90cupscupboard.xml")
    #w.readFile("../../../data/robots/tx90ball.rob")
    return w

def test_trajectory_editing():
    w = make_world()
    r = w.robot(0)
    q0 = r.getConfig()
    r.randomizeConfig()
    qrand = r.getConfig()
    r.setConfig(q0)
    r.randomizeConfig()
    qrand2 = r.getConfig()
    r.setConfig(q0)

    traj = SE3Trajectory([0,1],[se3.identity(),se3.identity()])
    saved,result = resource.edit("se3 traj",traj,'SE3Trajectory','Some spatial trajectory',world=w,referenceObject=r.link(7))

    traj = Trajectory([0,1],[[0,0,0],[0,0,1]])
    saved,result = resource.edit("point traj",traj,'auto','Some point trajectory',world=w,referenceObject=r.link(7))

    traj = RobotTrajectory(r,[0,1,2],[q0,qrand,qrand2])
    saved,result = resource.edit("robot traj",traj,'auto','Random robot trajectory',world=w,referenceObject=r)

def test_geometry_editing():
    w = make_world()
    g = GeometricPrimitive()
    g.setPoint([-0.5,0,1])
    saved,result = resource.edit("Point",g,world=w)
    print(result)
    g.setSphere([-0.5,0,1],0.3)
    saved,result = resource.edit("Sphere",g,world=w)
    print(result)
    g.setAABB([-0.5,0,1],[-0.2,0.1,1.3])
    saved,result = resource.edit("AABB",g,world=w)
    print(result)
    g.setBox([-0.5,0,1],so3.identity(),[0.4,0.4,0.4])
    saved,result = resource.edit("Box",g,world=w)
    print(result)

if __name__ == '__main__':
    test_trajectory_editing()
    test_geometry_editing()
    vis.kill()
    