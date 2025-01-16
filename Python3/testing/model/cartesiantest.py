import klampt
from klampt.model.cartesian_trajectory import cartesian_move_to
from klampt.model.trajectory import path_to_trajectory,execute_trajectory
from klampt.model import config,ik
from klampt.math import vectorops
from klampt import vis
import time
import random

world = klampt.WorldModel()
world.readFile("../../data/robots/tx90l.rob")
robot = world.robot(0)
robot.randomizeConfig()

sim = klampt.Simulator(world)
controller = sim.controller(0)


vis.add("world",world)
vis.show()

def move_random():
    q0 = robot.getConfig()
    vis.lock()
    # Now we set up a target
    ee_link = robot.numLinks()-1
    T0 = robot.link(ee_link).getTransform()
    target = vectorops.add(T0[1],[random.uniform(-0.2,0.2),random.uniform(-0.2,0.2),random.uniform(-0.2,0.2)])
    goal = ik.objective(robot.link(ee_link),R=T0[0],t=target)
    vis.add("target",target,color=(1,0,0,1),size=5.0)
    # Calling this function will generate a path from the current e.e. transform to goal
    path = cartesian_move_to(robot,goal)
    if path is None:
        print("Couldn't find a path!")
    else:
        # Now the path can be executed on a controller... note that it's untimed,
        # so a little work needs to be done to make it timed.  The path_to_trajectory
        # utility function helps a lot here!  It has many options so please consult
        # the documentation...
        traj = path_to_trajectory(path,smoothing=None,timing='Linf')
        speed = 1.0   #can vary the execution speed here or in path_to_trajectory.
        execute_trajectory(traj,controller,speed=speed)
    vis.unlock()
    robot.setConfig(q0)

vis.addAction(move_random,"Random dir",'r')

dt = 1.0/30.0
while vis.shown():
    #advance the simulation
    vis.lock()
    sim.simulate(dt)
    sim.updateWorld()
    vis.unlock()
    time.sleep(0.01)
