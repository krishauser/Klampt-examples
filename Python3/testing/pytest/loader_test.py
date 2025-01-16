from klampt.io import loader
import time

def doload(type,fn):
    t0 = time.time()
    res = loader.load(type,fn)
    t1 = time.time()
    print("Time to load",fn,":",t1-t0)
    return res

def dosave(obj,type,fn):
    t0 = time.time()
    res = loader.save(obj,type,fn)
    t1 = time.time()
    print("Time to save",fn,":",t1-t0)
    return res

def test_save(tmp_path):
    traj = doload('auto','https://raw.githubusercontent.com/krishauser/Klampt-examples/master/data/motions/athlete_flex.path')
    dosave(traj,'json','temp_traj.json')
    traj2 = doload('auto','temp_traj.json')
    assert len(traj.times) == len(traj2.times)
    assert len(traj.milestones) == len(traj2.milestones)

def test_url_load():
    doload('Trajectory','https://raw.githubusercontent.com/krishauser/Klampt-examples/master/data/motions/athlete_flex.path')
    doload('auto','https://raw.githubusercontent.com/krishauser/Klampt-examples/master/data/motions/athlete_flex.path')

    doload('Geometry3D','https://raw.githubusercontent.com/krishauser/Klampt-examples/master/data/terrains/plane.off')
    doload('auto','https://raw.githubusercontent.com/krishauser/Klampt-examples/master/data/terrains/plane.off')

if __name__ == '__main__':
    test_url_load()
