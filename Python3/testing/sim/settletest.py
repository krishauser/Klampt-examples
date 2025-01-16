#!/usr/bin/python

import sys
from klampt import *
from klampt import vis
from klampt.math import vectorops,so3
from klampt.sim import settle
import time
import math
import random

if __name__ == "__main__":
    print("settletest.py: This example demonstrates how to run the settling process")
    
    #creates a world and loads all the items on the command line
    world = WorldModel()
    fn = "~/Klampt-examples/data/tx90cups.xml"
    if len(sys.argv) > 1:
        fn = sys.argv[1]
    res = world.readFile(fn)
    if not res:
        raise RuntimeError("Unable to load model "+fn)

    vis.add("world",world)
    vis.show()
    if world.numRobots() > 0:
        vis.lock()
        t0 = time.time()
        linkno = world.robot(0).numLinks()-1
        T = settle.settle(world,world.robot(0).link(linkno),forcedir=(0,0,-2),perturb=0.0,settletol=1e-3,debug=False)
        t1 = time.time()
        vis.unlock()
        print("Settling robot link took time",t1-t0)
        input("Press enter to continue")
        vis.lock()

    for i in range(world.numRigidObjects()):
        obj = world.rigidObject(i)
        obj.setTransform(so3.sample(),obj.getTransform()[1])
    vis.unlock()
    
    for i in range(world.numRigidObjects()):
        vis.lock()

        obj = world.rigidObject(i)
        t0 = time.time()
        T,touched = settle.settle(world,obj,perturb=0.0,orientationDamping=0,settletol=1e-3,debug=False)
        obj.setTransform(*T)
        t1 = time.time()

        vis.unlock()
        print("Settling object",i,"took time",t1-t0)
        input("Press enter to continue")
    while vis.shown():
        time.sleep(0.01)
    vis.kill()