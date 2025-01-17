from klampt import WorldModel, Geometry3D
from klampt import vis
from klampt.model.trajectory import Trajectory,SE3Trajectory,RobotTrajectory
from klampt.vis import editors,colorize

def make_world():
    w = WorldModel()
    w.readFile("../../../data/tx90cupscupboard.xml")
    #w.readFile("../../../data/robots/tx90ball.rob")
    return w

def test_debug():
    w = make_world()
    r = w.robot(0)
    q0 = r.getConfig()
    r.randomizeConfig()
    qrand = r.getConfig()
    r.setConfig(q0)
    r.randomizeConfig()
    qrand2 = r.getConfig()
    r.setConfig(q0)
    
    vis.debug(w.robot(0),centerCamera=True)
    g = Geometry3D()
    g.loadFile("../../../data/objects/srimugsmooth.off")
    vis.debug(g,centerCamera=True)

    pt = [0,0,2]
    traj = Trajectory([0,1,2],[[0,0,2],[1,0,2],[1,1,2]])
    vis.debug('qrand',[qrand,qrand2,q0],{'color':[1,0,0,0.5]},pt,world=w)
    vis.debug('qrand',qrand,{'color':[1,0,0,0.5]},pt,world=w,centerCamera=r.link(6))
    vis.debug('qrand',qrand,{'color':[1,0,0,0.5]},world=w,followCamera=r.link(6))
    vis.debug('qrand',qrand,{'color':[1,0,0,0.5]},pt=pt,world=w,animation=traj)
    vis.debug('qrand',qrand,{'color':[1,0,0,0.5]},pt=pt,world=w,centerCamera='pt')
    vis.debug('qrand',qrand,{'color':[1,0,0,0.5]},pt=pt,world=w,animation=traj,followCamera='pt')

    milestones = []
    for i in range(5):
        r.randomizeConfig()
        milestones.append(r.getConfig())
    r.setConfig(q0)
    qtraj = RobotTrajectory(r,[0,1,2,3,4],milestones)
    vis.debug('qrand',qrand,{'color':[1,0,0,0.5]},world=w,animation=qtraj)
    #this doesn't work -- qrand is not being tracked
    vis.debug('qrand',qrand,{'color':[1,0,0,0.5]},world=w,animation=qtraj,followCamera=r.link(6))

if __name__ == '__main__':
    test_debug()
    vis.kill()