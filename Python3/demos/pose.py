#!/usr/bin/python

import sys
import klampt
from klampt import vis
from klampt.math import vectorops
from klampt.model.trajectory import RobotTrajectory
from klampt.control import StepContext,RobotInterfaceBase,RobotInterfaceCompleter,TimedLooper
from klampt.control.robotinterfaceutils import MultiprocessingRobotInterface,make_from_file
from klampt.control.interop import RobotInterfacetoVis
from klampt.control.simrobotinterface import *
import time

from kbdrive import load_world_and_interface

if __name__ == "__main__":
    print("pose_RIL.py: This example demonstrates how to pose a robot (via Robot Interface Layer)")
    if len(sys.argv)<=1:
        print("USAGE: python pose.py world_file(s) [controller_file]")
        print("   OR")
        print("python pose.py robotinfo_file [world_files]")
        print()
        print('try "python pose.py ../../data/tx90cuptable.xml"')
        exit()

    world, interface = load_world_and_interface(sys.argv[1:])
    if interface is None:    
        sim = klampt.Simulator(world)
        interface = RobotInterfaceCompleter(SimFullControlInterface(sim.controller(0),sim))
    elif not interface.properties.get('complete',False):
        interface = RobotInterfaceCompleter(interface)   #wrap the interface with a software emulator
    interface._klamptModel = world.robot(0)
    interface._worldModel = world
    if not interface.initialize():
        print("Robot interface",interface,"Could not be initialized")
        exit(1)

    interface.startStep()
    cqsns = interface.sensedPosition()
    world.robot(0).setConfig(interface.configToKlampt(cqsns))
    interface.endStep()

    target = None
    trajectory = None
    def do_move():
        global target
        target = vis.getItemConfig("ghost")
    vis.add("world",world)
    vis.add("ghost",world.robot(0).getConfig())
    vis.hide(vis.getItemName(world.robot(0)))
    vis.edit("ghost")
    vis.addAction(do_move,"Move to posed config",' ')
    visplugin = RobotInterfacetoVis(interface)
    vis.show()
    dt = 1.0/interface.controlRate()
    looper = TimedLooper(dt)   #when placed as the while loop guard, looper will try to keep the loop running @ 1/dt Hz
    iters = 0
    while vis.shown() and looper: 
        iters += 1
        #grab the editor's configuration and send to the robot(s)
        vis.lock()
        qdest = vis.getItemConfig("ghost")
        vis.unlock()
        with StepContext(interface):
            cqsns = interface.sensedPosition()
            cqcmd = interface.commandedPosition()
            #update the trajectory overlay
            if cqcmd is not None and cqcmd[0] is not None:
                qcmd = interface.configToKlampt(cqcmd)
                qmin,qmax = world.robot(0).getJointLimits()
                for i in range(len(qcmd)):
                    if qcmd[i] < qmin[i] or qcmd[i] > qmax[i]:
                        print("Joint",world.robot(0).link(i).getName(),"Out of limits?",qcmd[i],"vs",qmin[i],qmax[i])
                vis.add("traj",RobotTrajectory(world.robot(0),milestones=[qcmd,qdest]).getLinkTrajectory(world.robot(0).numLinks()-1,0.1))
            if target is not None:
                interface.moveToPosition(interface.configFromKlampt(target))
                target = None
            visplugin.update()
        #give visualizer a chance to update even when controller is running fast
        time.sleep(0.01)
    interface.close()
    vis.kill()
