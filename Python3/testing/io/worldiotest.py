from pytest import fixture
import os
import time
from klampt import *

WORLD_FN = "../../../data/baxter_apc.xml"
SAVE_DIR = 'worldio_tmp'
N = 100

def test_save_world():
    w0 = WorldModel()
    assert w0.readFile(WORLD_FN)
    w0.saveFile(os.path.join(SAVE_DIR,"temp_world.xml"))
    w1 = WorldModel()
    assert w1.readFile(os.path.join(SAVE_DIR,"temp_world.xml")),"Could not load previously saved world XML file"

def test_many_world():
    w0 = WorldModel()
    assert w0.readFile(WORLD_FN)
    w0.saveFile(os.path.join(SAVE_DIR,"temp_world.xml"))
    #this line takes all the geometries out of the cache
    #w0 = None

    t0 = time.time()
    for i in range(N):
        world = WorldModel()
        world.readFile(WORLD_FN)
    t1 = time.time()
    test1time = t1-t0

    w1 = WorldModel()
    assert w1.readFile(os.path.join(SAVE_DIR,"temp_world.xml")),"Could not load previously saved world XML file"
    #this line takes all the geometries out of the cache
    #w1 = None
    t0 = time.time()
    for i in range(N):
        world = WorldModel()
        assert world.readFile(os.path.join(SAVE_DIR,"temp_world.xml")),"Could not load previously saved world XML file"
    t1 = time.time()
    test2time = t1-t0
    print("Time to load",N,"worlds, raw:",test1time)
    print("Time to load",N,"worlds, after saving:",test2time)

if __name__ == '__main__':
    test_save_world()