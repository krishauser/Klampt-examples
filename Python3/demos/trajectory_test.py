#!/usr/bin/python

import sys
from klampt import *
from klampt import vis
from klampt.io import resource
from klampt.model import coordinates
from klampt.math import vectorops,so3,se3
from klampt.model import trajectory
import random
import time
import math

if __name__ == "__main__":
    print("trajectorytest.py: This example demonstrates several types of trajectory")
    if len(sys.argv)<=1:
        print("USAGE: trajectorytest.py [robot or world file]")
        print("With no arguments, shows some 3D trajectories")
        
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
        vis.animate("xform2",traj3)
        
        vis.animate("xform",traj)

        vis.setAttribute("xform_milestones","width",1.0)
        vis.setAttribute("xform spline","width",1.0)
        vis.setAttribute("xform bezier","width",1.0)
    else:
        #creates a world and loads all the items on the command line
        world = WorldModel()
        for fn in sys.argv[1:]:
            res = world.readFile(fn)
            if not res:
                raise RuntimeError("Unable to load model "+fn)

        #add the world to the visualizer
        robot = world.robot(0)
        vis.add("robot",robot)
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

        save,traj.milestones = resource.edit("trajectory",traj.milestones,world=world)
        vis.animate("robot",traj)
    
    #pop up the window to show the trajectory
    vis.spin(float('inf'))

    if len(sys.argv)>1:
        vis.animate("robot",None)
        sim = Simulator(world)
        sim.simulate(0)
        trajectory.execute_path(traj.milestones,sim.controller(0))
        vis.setWindowTitle("Simulating path using trajectory.execute_trajectory")
        if vis.multithreaded():
            #for some tricky Qt reason, need to sleep before showing a window again
            #Perhaps the event loop must complete some extra cycles?
            time.sleep(0.01)
            vis.show()
            t0 = time.time()
            while vis.shown():
                #print "Time",sim.getTime()
                sim.simulate(0.01)
                if sim.controller(0).remainingTime() <= 0:
                    print("Executing timed trajectory")
                    trajectory.execute_trajectory(traj,sim.controller(0),smoothing='pause')
                vis.setItemConfig("robot",sim.controller(0).getCommandedConfig())
                t1 = time.time()
                time.sleep(max(0.01-(t1-t0),0.0))
                t0 = t1
        else:
            data = {'next_sim_time':time.time()}
            def callback():
                if time.time() >= data['next_sim_time']:
                    sim.simulate(0.01)
                    if sim.controller(0).remainingTime() <= 0:
                        print("Executing timed trajectory")
                        trajectory.execute_trajectory(traj,sim.controller(0),smoothing='pause')
                    vis.setItemConfig("robot",sim.controller(0).getCommandedConfig())
                    data['next_sim_time'] += 0.01
            vis.loop(callback=callback,setup=vis.show)
    print("Ending vis.")
    vis.kill()
