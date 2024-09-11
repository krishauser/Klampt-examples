#!/usr/bin/python

import sys
from klampt import *
from klampt import vis
from klampt.io import resource
from klampt.model import coordinates
from klampt.math import vectorops,so3,se3
from klampt.model import trajectory
from klampt.control.robotinterface import RobotInterfaceBase
from klampt.control.robotinterfaceutils import StepContext, RobotInterfaceCompleter, make_from_file
from klampt.control.utils import TimedLooper
import random
import time
import math

def basic_trajectory_test():
    #add a point to the visualizer and animate it
    point = coordinates.addPoint("point")
    vis.add("point",point)
    traj = trajectory.Trajectory()
    x = -2
    for i in range(10):
        traj.times.append(i)
        traj.milestones.append([x + random.uniform(0.1,0.3),0,random.uniform(-1,1)])
        x = traj.milestones[-1][0]

    traj2 = trajectory.HermiteTrajectory()
    traj2.makeMinTimeSpline(traj.milestones,vmax=[2,2,2],amax=[4,4,4])
    vis.add("point minTime",traj2.discretize(0.05),color=(0,0,1,1))

    traj3 = trajectory.HermiteTrajectory()
    traj3.makeMinTimeSpline(traj.milestones,[[0,0,0]]*len(traj.milestones),vmax=[2,2,2],amax=[8,8,8])
    for m in traj3.milestones:
        m[1] = 0.25
    vis.add("point mintime, nonlinear",traj3.discretize(0.05),color=(0.2,0.2,1,1))

    traj4 = trajectory.HermiteTrajectory()
    traj4.makeSpline(traj)
    for m in traj4.milestones:
        m[1] = 0.5
    vis.add("point spline",traj4.discretize(0.05),color=(0.4,0.4,1,1))

    traj5 = trajectory.HermiteTrajectory()
    traj5.makeSpline(traj,preventOvershoot=False)
    for m in traj5.milestones:
        m[1] = 0.75
    vis.add("point spline, overshoot allowed",traj5.discretize(0.05),color=(0.6,0.6,1,1))

    traj6 = trajectory.HermiteTrajectory()
    traj6.makeBezier([0,3,6,9],traj.milestones)
    for m in traj6.milestones:
        m[1] = 1
    vis.add("point bezier",traj6.discretize(0.05),color=(0.8,0.8,1,1))

    #which one to animate?
    vis.animate("point",traj2)

    #add a transform to the visualizer and animate it
    xform = vis.add("xform",se3.identity())
    traj = trajectory.SE3Trajectory()
    x = 0
    for i in range(10):
        rrot = so3.sample()
        rpoint = [x + random.uniform(0.1,0.3),0,random.uniform(-1,1)]
        x = rpoint[0]
        traj.milestones.append(rrot+rpoint)
    traj.times = list(range(len(traj.milestones)))
    vis.add("xform_milestones",traj,color=(1,1,0,1))

    traj2 = trajectory.SE3HermiteTrajectory()
    traj2.makeSpline(traj)
    for m in traj2.milestones:
        m[10] = 0.25
    vis.add("xform spline",traj2.discretize(0.05),color=(1,0.8,0,1))
    vis.add("xform2",se3.identity())
    vis.animate("xform2",traj2)

    traj3 = trajectory.SE3HermiteTrajectory()
    traj3.makeBezier([0,3,6,9],traj.milestones)
    for m in traj3.milestones:
        m[10] = 0.5
    vis.add("xform bezier",traj3.discretize(0.05),color=(1,0.6,0,1))
    vis.add("xform3",se3.identity())
    vis.animate("xform3",traj3)
    
    vis.animate("xform",traj)

    vis.setAttribute("xform_milestones","width",1.0)
    vis.setAttribute("xform spline","width",1.0)
    vis.setAttribute("xform bezier","width",1.0)
    vis.loop()
    vis.kill()

def edit_trajectory_test(world):
    #add the world to the visualizer
    vis.add("world",world)
    robot = world.robot(0)
    robot_path = vis.getItemName(robot)
    traj = trajectory.RobotTrajectory(robot)
    qmin,qmax = robot.getJointLimits()
    q0 = robot.getConfig()
    for i in range(robot.numLinks()):
        if math.isinf(qmin[i]) or math.isinf(qmax[i]):
            #don't animate floating base
            continue
        traj.times.append(len(traj.times)*0.5)
        q = q0[:]
        q[i] = qmin[i]
        traj.milestones.append(q)
        
        traj.times.append(len(traj.times)*0.5)
        q[i] = qmax[i]
        traj.milestones.append(q)

    save,traj = resource.edit("trajectory",traj,world=world)
    vis.animate(robot_path,traj)
    vis.loop()
    vis.kill()

def controller_trajectory_test(world : WorldModel, interface_fn : str):
    robot = world.robot(0)
    
    #extract current configuration
    interface = make_from_file(interface_fn, robot)
    if not interface.properties.get('complete',False):
        interface = RobotInterfaceCompleter(interface)
    if not interface.initialize():
        sys.exit(1)
    with StepContext(interface):
        try:
            q = interface.commandedPosition()
        except Exception:
            q = interface.sensedPosition()
    interface.close()

    #edit configuration (close interface while this goes on)
    qklampt = interface.configToKlampt(q)
    traj = trajectory.RobotTrajectory(robot)
    traj.times = [0.0,1.0]
    traj.milestones = [qklampt,qklampt]
    save,traj = resource.edit("trajectory",traj,world=world)

    #create a new interface
    interface = make_from_file(interface_fn, robot)
    if not interface.properties.get('complete',False):
        interface = RobotInterfaceCompleter(interface)
    if not interface.initialize():
        sys.exit(1)

    vis.add("world",world)
    robot_path = vis.getItemName(robot)
    vis.setWindowTitle("Simulating path using trajectory.execute_trajectory")
    if vis.multithreaded():
        #for some tricky Qt reason, need to sleep before showing a window again
        #Perhaps the event loop must complete some extra cycles?
        time.sleep(0.01)
        vis.show()
        dt = 1.0/interface.controlRate()
        looper = TimedLooper(dt)
        t0 = time.time()
        while vis.shown() and looper:
            #print "Time",sim.getTime()
            with StepContext(interface):
                vcmd = interface.commandedVelocity()
                if not interface.isMoving():
                    trajectory.execute_trajectory(traj,interface,smoothing='pause')
                    if traj.times[0] == 0:
                        traj.times = [t+1 for t in traj.times]
                qcontroller = interface.commandedPosition()
                qklampt = interface.configToKlampt(qcontroller)
                vis.setItemConfig(robot_path,qklampt)
            time.sleep(0.01) #needed to ensure visualizer refreshes
    else:
        dt = 1.0/interface.controlRate()
        data = {'next_sim_time':time.time()}
        def callback():
            t = time.time()
            if t >= data['next_sim_time']:
                with StepContext(interface):
                    if not interface.isMoving():
                        print("Executing timed trajectory")
                        trajectory.execute_trajectory(traj,interface,smoothing='pause')
                        if traj.times[0] == 0:
                            traj.times = [t+1 for t in traj.times]
                    qcontroller = interface.commandedPosition()
                    qklampt = interface.configToKlampt(qcontroller)
                    vis.setItemConfig(robot_path,qklampt)
                data['next_sim_time'] += dt
                if data['next_sim_time'] < t:
                    data['next_sim_time'] = t
        vis.loop(callback=callback,setup=vis.show)
    vis.kill()


if __name__ == "__main__":
    print("trajectorytest.py: This example demonstrates several types of trajectory")
    if len(sys.argv)<=1:
        print("USAGE: trajectorytest.py [robot or world file] [controller file]")
        print("With no arguments, shows some 3D trajectories")
        
        basic_trajectory_test()
    else:

        #creates a world and loads all the items on the command line
        interface_fn = None
        world = WorldModel()
        for fn in sys.argv[1:]:
            if fn.endswith('.py') or fn.endswith('.pyc') or fn.startswith('klampt.'):
                interface_fn = fn
            else:
                res = world.readFile(fn)
                if not res:
                    raise RuntimeError("Unable to load model "+fn)
        
        if interface_fn is None:
            edit_trajectory_test(world)
        else:
            controller_trajectory_test(world,interface_fn)
    
