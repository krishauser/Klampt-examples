from klampt import *
from klampt import vis
from klampt.io import loader
import time
import math
import sys

if len(sys.argv) > 1:
    fn = sys.argv[1]
else:
    fn = "../../data/robots/kinova_gen3_7dof.urdf"

if len(sys.argv) > 2:
    config = sys.argv[2]
else:
    config = None

print("-------------------------------------------------------------------")
print("exercise_joints.py: This example moves between all of a robot's")
print("joint extrema.")
print()
print("Usage: python exercise_joints.py [robot_or_world_file] [config]")
print()
print("You can also specify 'random' for the config.")
print()
print("-------------------------------------------------------------------")

w = WorldModel()
if not w.readFile(fn):
    print("Unable to read file",fn)
    exit(1)

r = w.robot(0)
if config == 'random':
    r.randomizeConfig()
elif config is not None:
    if config.endswith('.config'):
        q = io.load(config,'Config')
    else:
        q = io.read('Config',config)
    r.setConfig(q)

vis.add("robot",r)
for i in range(1,w.numRobots()):
    vis.add(w.robot(i).getName(),w.robot(i))
for i in range(0,w.numRigidObjects()):
    vis.add(w.rigidObject(i).getName(),w.rigidObject(i))
for i in range(0,w.numTerrains()):
    vis.add(w.terrain(i).getName(),w.terrain(i))
vis.addPlot("joints")
vis.addPlotItem("joints","robot")
vis.show()

d0 = [r.driver(i).getValue() for i in range(r.numDrivers())]
qmin,qmax = r.getJointLimits()
index = 0
period = 5
period_start = time.time()
while vis.shown():
    t = time.time()
    u = (t - period_start)/period
    
    driver = r.driver(index)
    if driver.getType()=='affine':
        dmin=float('inf')
        dmax=-float('inf')
        s,c = driver.getAffineCoeffs()
        for l in driver.getAffectedLinks():
            if s > 0:
                dmin = min(dmin,(qmin[l]-c)/s)
                dmax = min(dmin,(qmax[l]-c)/s)
            elif s < 0:
                dmin = min(dmin,(qmax[l]-c)/s)
                dmax = min(dmin,(qmin[l]-c)/s)
    else:
        dmin=qmin[driver.getAffectedLink()]
        dmax=qmax[driver.getAffectedLink()]
        if dmax - dmin > math.pi*2:
            if d0[index] > math.pi:
                dmin = 0
                dmax = math.pi*2
            else:
                dmin = -math.pi
                dmax = math.pi
    uswitch1 = (dmax - d0[index])/(dmax-dmin)*0.5
    uswitch2 = uswitch1 + 0.5
    vis.lock()
    if u < uswitch1:
        driver.setValue(d0[index]+u/uswitch1*(dmax-d0[index]))
    elif u < uswitch2:
        driver.setValue(dmax+(u-uswitch1)/0.5*(dmin-dmax))
    elif u < 1:
        driver.setValue(dmin+(u-uswitch1-0.5)/(1-uswitch1-0.5)*(d0[index]-dmin))
    else:
        driver.setValue(d0[index])
    vis.setItemConfig("robot",r.getConfig())
    vis.unlock()
    if u > 1:
        index += 1
        index = index % r.numDrivers()
        period_start += period
    time.sleep(0.01)
vis.kill()
