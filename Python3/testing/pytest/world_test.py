from pytest import fixture
import os
import time
from klampt import *

@fixture
def world_fn() -> str:
    return "../../../data/baxter_apc.xml"
    
@fixture
def N():
    return 100

def test_save_world(world_fn, tmp_path):
    w0 = WorldModel()
    assert w0.readFile(world_fn)
    w0.saveFile(os.path.join(tmp_path,"temp_world.xml"))
    w1 = WorldModel()
    assert w1.readFile(os.path.join(tmp_path,"temp_world.xml")),"Could not load previously saved world XML file"

def test_many_world(world_fn, tmp_path, N):
    w0 = WorldModel()
    assert w0.readFile(world_fn)
    w0.saveFile(os.path.join(tmp_path,"temp_world.xml"))
    #this line takes all the geometries out of the cache
    #w0 = None

    t0 = time.time()
    for i in range(N):
        world = WorldModel()
        world.readFile(world_fn)
    t1 = time.time()
    test1time = t1-t0

    w1 = WorldModel()
    assert w1.readFile(os.path.join(tmp_path,"temp_world.xml")),"Could not load previously saved world XML file"
    #this line takes all the geometries out of the cache
    #w1 = None
    t0 = time.time()
    for i in range(N):
        world = WorldModel()
        assert world.readFile(os.path.join(tmp_path,"temp_world.xml")),"Could not load previously saved world XML file"
    t1 = time.time()
    test2time = t1-t0
    print("Time to load",N,"worlds, raw:",test1time)
    print("Time to load",N,"worlds, after saving:",test2time)

if __name__ == '__main__':
    test_many_world('../../../data/baxter_apc.xml','.',100)