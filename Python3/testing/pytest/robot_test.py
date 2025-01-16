from pytest import fixture
from klampt import *
from klampt import vis
import sys

@fixture
def world() -> WorldModel:
    w = WorldModel()
    w.readFile("../../../data/baxter_apc.xml")
    return w

def test_reduce(world):
    assert world.numRobots() > 0
    world.makeRobot("reduced")
    dofmap = world.robot(1).reduce(world.robot(0))
    assert len(dofmap) == world.robot(0).numLinks()
    for i,j in enumerate(dofmap):
        if j < 0:
            continue
        assert j >= 0 and j < world.robot(1).numLinks()
        assert world.robot(0).link(i).getName() == world.robot(1).link(j).getName()
        