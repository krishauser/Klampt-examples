from klampt import WorldModel
from klampt import vis
from klampt.model.trajectory import Trajectory,SE3Trajectory,RobotTrajectory

def make_world():
    w = WorldModel()
    w.readFile("../../../data/tx90cupscupboard.xml")
    #w.readFile("../../../data/robots/tx90ball.rob")
    return w

def test_trajectory_vis():
    w = make_world()
    r = w.robot(0)
    q0 = r.getConfig()
    r.randomizeConfig()
    qrand = r.getConfig()
    r.setConfig(q0)
    
    #add a "world" item to the scene manager
    vis.add("world",w)
    #show qrand as a ghost configuration in transparent red
    vis.add("qrand",qrand,color=(1,0,0,0.5))
    #show a Trajectory between q0 and qrand
    traj = RobotTrajectory(r,[0,1],[q0,qrand])
    vis.add("path_to_qrand",traj)
    #show link trajectory
    eetraj = traj.getLinkTrajectory(6,discretization=0.01)
    r.setConfig(q0)
    vis.add("ee_traj",eetraj)

    #launch the vis loop and window
    vis.show()
    vis.spin(float('inf'))
    vis.kill()

if __name__ == '__main__':
    test_trajectory_vis()