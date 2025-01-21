#!/usr/bin/python

import sys
from klampt import *
from klampt.model.robotinfo import RobotInfo
from klampt.math import vectorops,so3,se3
from klampt import vis
from klampt.vis.glrobotprogram import *

#Sphero local axes: [0,0,1] is "turn", [1,0,0] is "drive"
#can be 'left','up','down','right', 'home', 'insert', 'end', and the function keys 'f1',...,'f12'.
keymap = {'up':(0,[-1,0,0]),'down':(0,[1,0,0]),'left':(0,[0,0,-1]),'right':(0,[0,0,1])}

class MyGLViewer(GLSimulationPlugin):
    def __init__(self,world):
        global keymap
        from functools import partial
        GLSimulationPlugin.__init__(self,world)
        self.keymap = keymap
        self.current_velocities = {}
        #Put your initialization code here
        sphero = RobotInfo.load("../../../robotinfo/sphero/sphero_sim.json")
        #initialize emulators
        self.sphero_emulators = []
        for r in range(world.numRobots()):
            sphero.configureSimulator(self.sim,r)
            self.sphero_emulators.append(self.sim.actuatorEmulators[r][0])

        for c in self.keymap:
            def setvel(d):
                self.current_velocities[d]=self.keymap[d]
            self.add_action(partial(setvel,c),"Move "+c,c)

    def control_loop(self):
        #Calculate the desired velocity for each robot by adding up all
        #commands
        rvels = [[0]*3 for r in range(self.world.numRobots())]
        for (c,(r,v)) in self.current_velocities.items():
            rvels[r] = vectorops.add(rvels[r],v)
        #send to the robot(s)
        for r in range(self.world.numRobots()):           
            self.sphero_emulators[r].process({'twist':rvels[r]},self.dt)
        return

    def keyboardupfunc(self,c,x,y):
        if c in self.current_velocities:
            del self.current_velocities[c]
        return


if __name__ == "__main__":
    print("sphero.py: This example demonstrates how to drive a custom Sphero simulator using keyboard input")
    if len(sys.argv)<=1:
        print("USAGE: sphero.py [world_file]")
        sys.argv = [sys.argv[0],'sphero_data/sphero.xml']
    world = WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)
    viewer = MyGLViewer(world)
    vis.run(viewer)
