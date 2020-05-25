from klampt.io import loader
import time

def testload(type,fn):
    t0 = time.time()
    res = loader.load(type,fn)
    t1 = time.time()
    print("Time to load",fn,":",t1-t0)
    return res

def testsave(obj,type,fn):
    t0 = time.time()
    res = loader.save(obj,type,fn)
    t1 = time.time()
    print("Time to save",fn,":",t1-t0)
    return res

testload('Trajectory','https://raw.githubusercontent.com/krishauser/Klampt-examples/master/data/motions/athlete_flex.path')
traj = testload('auto','https://raw.githubusercontent.com/krishauser/Klampt-examples/master/data/motions/athlete_flex.path')
testsave(traj,'json','temp_traj.json')
traj2 = testload('auto','temp_traj.json')

testload('Geometry3D','https://raw.githubusercontent.com/krishauser/Klampt-examples/master/data/terrains/plane.off')
testload('auto','https://raw.githubusercontent.com/krishauser/Klampt-examples/master/data/terrains/plane.off')

