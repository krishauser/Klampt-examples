#!/usr/bin/python

import sys
import klampt
from klampt.vis.glinterface import GLPluginInterface
from klampt import vis
from klampt.math import vectorops
from klampt.model.robotinfo import RobotInfo
from klampt.control import StepContext,RobotInterfaceBase,RobotInterfaceCompleter,TimedLooper
from klampt.control.robotinterfaceutils import make_from_file
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
        
        self.controller_dt = 1.0/interface.controlRate()
        self.vis_dt = 1.0/30.0
        self.num_steps = 0
        #Put your initialization code here
        
    def idle(self):
        if not self.interface.properties.get('asynchronous',False):
            #take extra controller steps if not asynchronous
            num_vis_steps = int(self.vis_dt / self.controller_dt)-1
            for i in range(num_vis_steps):
                with StepContext(self.interface):
                    self.num_steps += 1
        
        #Calculate the desired velocity for each robot by adding up all
        #commands
        rvels = [[0]*self.world.robot(r).numDrivers() for r in range(self.world.numRobots())]
        for (c,(r,v)) in self.current_velocities.items():
            rvels[r] = vectorops.add(rvels[r],v)
        #print(rvels)
        #send to the robot(s)
        with StepContext(self.interface):
            if self.num_steps > 0:
                ttl = 0.1
                self.interface.setVelocity(rvels[0],ttl)
            self.world.robot(0).setConfig(self.interface.configToKlampt(self.interface.sensedPosition()))
            self.visplugin.update()
            self.num_steps += 1
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

def load_world_and_interface(args):
    """Utility function.  Loads a WorldModel and RobotInterfaceBase
    from command line args.
    
    Returns:
        (world,interface). Sets interface=None if no controller is
        specified; user will need to set up their desired default.
    """
    world = klampt.WorldModel()
    robotinfo = RobotInfo('robot')
    for fn in args:
        if fn.endswith('.py') or fn.endswith('.pyc') or fn.startswith('klampt.'):
            robotinfo.controllerFile = fn
        elif fn.endswith('.json'):
            robotinfo = RobotInfo.load(fn)
        else:
            res = world.readFile(fn)
            if not res:
                raise RuntimeError("Unable to load model "+fn)
    if robotinfo.modelFile is None and world.numRobots() == 0:
        print("No robots loaded")
        exit(1)
    if world.numRobots()  != 0:
        robotinfo.robotModel = world.robot(0)
    else:
        robot = robotinfo.klamptModel()
        world.add(robotinfo.name,robot)
    if robotinfo.controllerFile is not None:
        interface = robotinfo.controller()
        return world,interface
    return world,None

    
if __name__ == "__main__":
    print("kbdrive.py: This example demonstrates how to drive a robot")
    print("(via Robot Interface Layer) using keyboard input")
    if len(sys.argv)<=1:
        print("USAGE: python kbdrive.py world_file(s) [controller_file]")
        print("   OR")
        print("python kbdrive.py robotinfo_file [world files]")
        exit()

    world,interface = load_world_and_interface(sys.argv[1:])
    if interface is None:
        #initialize simulated controller by default
        sim = klampt.Simulator(world)
        interface = RobotInterfaceCompleter(SimFullControlInterface(sim.controller(0),sim))
    elif not interface.properties.get('complete',False):
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
