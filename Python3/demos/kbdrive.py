#!/usr/bin/python

import sys
import klampt
from klampt.vis.glinterface import GLPluginInterface
from klampt import vis
from klampt.math import vectorops
from klampt.control.robotinterfaceutils import StepContext,RobotInterfaceCompleter,make_from_file
from klampt.control.interop import RobotInterfacetoVis
from klampt.control.simrobotinterface import *
import time

#FOR DEFAULT JOINT-BY-JOINT KEYMAP: set keymap=None
keymap = None

SPEED = 1

#FOR CUSTOM KEYMAPS: set up keymap to define how keys map to velocities.
#keymap is a map from key name to (robot index,velocity vector) pairs.
#Key names can either be single keys or names of special keys
#'left','up','down','right', 'home', 'insert', 'end', and the function keys 'f1',...,'f12'.
#keymap = {'up':(0,[0,1]),'down':(0,[0,-1]),'left':(0,[-1,0]),'right':(0,[1,0])}

def build_default_keymap(robot):
    """builds a default keymape: 1234567890 increases values of DOFs 1-10
    of robot 0.  qwertyuiop decreases values."""
    up = '1234567890'
    down = 'qwertyuiop'
    res = {}
    for i in range(min(robot.numDrivers(),10)):
        #up velocity
        vel = [0]*robot.numDrivers()
        vel[i] = SPEED
        res[up[i]] = (0,vel)
        #down velocity
        vel = vectorops.mul(vel,-1)
        res[down[i]] = (0,vel)
    return res



class MyGLViewer(GLPluginInterface):
    def __init__(self,interface):
        global keymap
        self.interface = interface
        self.robot_model = interface.klamptModel()
        GLPluginInterface.__init__(self)
        self.world = interface._worldModel
        if keymap == None:
            keymap = build_default_keymap(self.robot_model)
        self.keymap = keymap
        self.current_velocities = {}
        self.visplugin = RobotInterfacetoVis(self.interface)
        self.wait = 0
        #Put your initialization code here
        
    def idle(self):
        t0 = time.time()
        self.wait += 1
        
        #Calculate the desired velocity for each robot by adding up all
        #commands
        rvels = [[0]*self.world.robot(r).numDrivers() for r in range(self.world.numRobots())]
        for (c,(r,v)) in self.current_velocities.items():
            rvels[r] = vectorops.add(rvels[r],v)
        #print(rvels)
        #send to the robot(s)
        with StepContext(self.interface):
            if self.wait > 1:
                ttl = 0.1
                self.interface.setVelocity(rvels[0],ttl)
            self.world.robot(0).setConfig(self.interface.configToKlampt(self.interface.sensedPosition()))
            self.visplugin.update()
        t1 = time.time()
        return True

    def print_help(self):
        self.window.program.print_help()
        print('Drive keys:',sorted(self.keymap.keys()))

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        if c in self.keymap:
            self.current_velocities[c]=self.keymap[c]
            return True
        elif c == '?':
            self.print_help()
            return True
        else:
            return GLPluginInterface.keyboardfunc(self,c,x,y)
        self.refresh()

    def keyboardupfunc(self,c,x,y):
        if c in self.current_velocities:
            del self.current_velocities[c]
        return


if __name__ == "__main__":
    print("kbdrive_RIL.py: This example demonstrates how to drive a robot")
    print("(via Robot Interface Layer) using keyboard input")
    if len(sys.argv)<=1:
        print("USAGE: kbdrive_ril.py [world_file(s)] [controller_file]")
        exit()

    world = klampt.WorldModel()
    interface_fn = None
    for fn in sys.argv[1:]:
        if fn.endswith('.py') or fn.endswith('.pyc'):
            interface_fn = fn
        else:
            res = world.readFile(fn)
            if not res:
                raise RuntimeError("Unable to load model "+fn)
    if world.numRobots() == 0:
        print("No robots loaded")
        exit(1)
    #initialize controller or simulated controller by default
    if interface_fn is None:    
        sim = klampt.Simulator(world)
        interface = RobotInterfaceCompleter(SimFullControlInterface(sim.controller(0),sim))
    else:
        try:
            interface = make_from_file(interface_fn,world.robot(0))
        except Exception:
            print("Quitting...")
            sys.exit(1)
        if not interface.properties.get('complete',False):
            interface = RobotInterfaceCompleter(interface)   #wrap the interface with a software emulator
    interface._klamptModel = world.robot(0)
    interface._worldModel = world
    if not interface.initialize():
        print("Robot interface",interface,"Could not be initialized")
        exit(1)
    viewer = MyGLViewer(interface)

    print() 
    print("**********************")
    print("       HELP")
    print("Press 's' to start simulating.")
    print("Use 1,2,..,0 to increase the robot's joints and q,w,...,p to reduce them.")
    print("**********************")
    print() 
    vis.add("world",world)
    vis.pushPlugin(viewer)
    vis.run()
