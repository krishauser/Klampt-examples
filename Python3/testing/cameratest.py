from klampt.robotsim import *
from klampt.math import se3,so3,vectorops
from klampt.model.trajectory import *
from klampt import vis
import sys
import time
import math
import random

w = WorldModel()
if len(sys.argv) > 1:
    if not w.loadFile(sys.argv[1]):
        print("Error loading",sys.argv[1])
        exit(1)
else:
    if not w.loadFile("../../data/simulation_test_worlds/sensortest.xml"):
        print("Error loading test world?")
        exit(1)

sim = Simulator(w)
robot = w.robot(0)
cam = sim.controller(0).sensor("rgbd_camera")
link = robot.link(robot.numLinks()-1)
amplitudes = [random.uniform(0,2) for i in range(robot.numLinks())]
phases = [random.uniform(0,math.pi*2) for i in range(robot.numLinks())]
periods = [random.uniform(0.5,2) for i in range(robot.numLinks())]
qmin,qmax = robot.getJointLimits()

vis.add("world",w)
vis.add("cam",cam)
vp = vis.getViewport()
vp.camera.rot[1] -= 0.5
vis.setViewport(vp)
default = vis.getViewport().get_transform()
print('x:',so3.apply(default[0],[1,0,0]))
print('y:',so3.apply(default[0],[0,1,0]))
print('z:',so3.apply(default[0],[0,0,1]))
print('offset:',default[1])
circle_points = []
npts = 50
times = []
for i in range(npts+1):
    angle = math.radians(360*i/npts)
    circle_points.append(se3.mul((so3.rotation([0,0,1],angle),[0,0,0]),default))
    times.append(i*20/npts)
circle_traj = SE3Trajectory(times,circle_points)
circle_traj.milestones[-1] = circle_traj.milestones[0]
circle_smooth_traj = SE3HermiteTrajectory()
circle_smooth_traj.makeSpline(circle_traj,loop=True)
R0 = so3.identity()
R1 = so3.rotation([0,0,1],math.pi/2)
dR0 = [0.0]*9

#TODO: for some reason the interpolation doesn't work very well...
#vis.add("Camera smooth traj",circle_smooth_traj.discretize_se3(0.1))
#for m in circle_smooth_traj.milestones:
#    T = m[:12]
#    vT = m[12:]
#    print("Orientation velocity",vT[:9])

#print("SMOOTH")
#for R in circle_smooth_traj.discretize_se3(0.1).milestones:
#    print(so3.apply(R,[0,1,0]))
vis.add("xform",se3.identity())
vis.animate("xform",circle_smooth_traj)
#vis.add("Camera traj",circle_traj.discretize(0.25))
vis.addAction(lambda:vis.followCamera(None),"stop folllowing")
vis.addAction(lambda:vis.followCamera(cam),"robot camera")
vis.addAction(lambda:vis.followCamera(("world",robot.getName(),link.getName()),True,False,True),"link, translate")
vis.addAction(lambda:vis.followCamera(("world",robot.getName(),link.getName()),False,True,True),"link, rotate")
vis.addAction(lambda:vis.followCamera(("world",robot.getName(),link.getName()),True,True,True),"link, full pose")
vis.addAction(lambda:vis.followCamera(("world",robot.getName(),link.getName()),True,False,False),"link, translate, nocenter")
vis.addAction(lambda:vis.followCamera(("world",robot.getName(),link.getName()),False,True,False),"link, rotate, nocenter")
vis.addAction(lambda:vis.followCamera(("world",robot.getName(),link.getName()),True,True,False),"link, full pose, nocenter")
vis.addAction(lambda:vis.followCamera(circle_traj,True,False),"circle, translate")
vis.addAction(lambda:vis.followCamera(circle_traj,False,True),"circle, rotate")
vis.addAction(lambda:vis.followCamera(circle_traj,True,True),"circle, full pose")

t0 = time.time()
vis.show()
while vis.shown():
    t = time.time()-t0
    q = [amp*math.sin(t/period - phase) for (amp,period,phase) in zip(amplitudes,periods,phases)]
    q = [min(b,max(v,a)) for (a,b,v) in zip(qmin,qmax,q)]
    vis.lock()
    robot.setConfig(q)
    vis.unlock()
    time.sleep(0.01)
vis.kill()
