"""
Test the problem of ur5 with 2 blocks as obstacles
"""
from __future__ import print_function
import sys
import time
import numpy as np
import copy
import math
import sys
import pickle as pkl
import matplotlib.pyplot as plt

from klampt import WorldModel, TriangleMesh, Geometry3D, TerrainModel
from klampt.model import ik, coordinates, config, cartesian_trajectory, trajectory, create
from klampt.io import resource
from klampt.plan import cspace, robotplanning
from klampt.plan.robotplanning import makeSpace
from klampt import vis
from klampt.math import vectorops as vo, so3
from klampt.plan.kinetrajopt import KineTrajOpt, TrajOptSettings


def main():
    world, robot = load_world()
    space = makeSpace(world, robot)
    collision_check = lambda: (space.selfCollision(robot.getConfig()))==False and (space.envCollision(robot.getConfig())==False)
    # q0, qf = compute_initial_final(world, robot)
    q0 = np.array([0.0, 0.04143683792522413, -1.183867130629673, 2.0680756463305556, 3.745524327531242, 3.6939852943821956e-06, 0.08270468308944955, 0.0])
    qf = np.array([0.0, 3.590769443639893, -0.8366423979290207, 1.5259988654642873, 5.59380779741848, 0.8157228966102285, 4.712401453217654, 0.0])
    robot.setConfig(q0)
    q0 = q0[1:-1]
    qf = qf[1:-1]
    link_index = [1, 2, 3, 4, 5, 6]
    geom_index = link_index
    obs_index = [0, 1]
    n_grid = 10  # resolution
    guess = np.linspace(q0, qf, n_grid)
    # guess += np.random.uniform(-0.1, 0.1, size=guess.shape)
    vis.add("world", world)
    # this one has no additional constraint so it should be easy to solve...
    config = TrajOptSettings(cvxpy_args={'solver': 'GUROBI'})
    trajopt = KineTrajOpt(world, robot, q0, qf, config=config, link_index=link_index, obs_index=obs_index, geom_index=geom_index)
    rst = trajopt.optimize(guess)
    sol = rst['sol']
    for i in range(1, n_grid):
        robot.setConfig([0] + sol[i].tolist() + [0])
        print(collision_check())
        vis.add('ghost%d' % i, [0] + list(sol[i]) + [0])
        vis.setColor('ghost%d' % i, 0, 1, 0, 0.5)
    vis.spin(float('inf'))


def load_world():
    world = WorldModel()
    create.primitives.box(0.5, 0.1, 0.5, [0.6, -0.2, 1.6], world=world, name='obstacle')  # z is 1.2 for classical setting
    create.primitives.box(0.5, 0.1, 0.5, [0.6, 0.25, 1.6], world=world, name='obstacle2')  # z used to be 1.6 for classical setting
    create.primitives.sphere(0.1, [-1, -1, 0], world=world, name='origin')
    create.primitives.sphere(0.1, [0, -1, 0], world=world, name='X')
    create.primitives.sphere(0.1, [-1, 1, 0], world=world, name='Y')
    fn = "./assets/ur5data/ur5Blocks.xml"
    res = world.readFile(fn)
    robot = world.robot(0)
    # change joint limits to avoid self collision
    qmin, qmax = robot.getJointLimits()
    qmin[2] = -math.pi / 2
    qmax[2] = 0
    qmin[3] = 0
    qmax[3] = math.pi
    robot.setJointLimits(qmin, qmax)
    return world, robot


def compute_initial_final(world, robot):
    """
    Initial point is fixed and given, target is set by
    1. rotate q0 a random angle between -pi/4, 3*pi/4
    2. random perturb ee position by 0.3 m
    If IK fails, simply abandon it.
    Exit when 10 plans are generated.
    """
    end_effector = robot.link(7)
    space = makeSpace(world, robot)
    # we want to the end effector to move from start to goal
    start = [-0.2, -0.50, 1.4]
    # solve IK for the start point
    unit_vec1 = [0.0, -0.1, 0.0]
    null = [0.0, 0.0, 0.0]
    unit_vec2 = [0.0, 0., -0.1]
    objective = ik.objective(end_effector, local = [[0,0,0], unit_vec1], world =[start,vo.madd(start, unit_vec2, 1)])
    collision_check = lambda: (space.selfCollision(robot.getConfig()))==False and (space.envCollision(robot.getConfig())==False)
    ikflag = ik.solve_global(objective, feasibilityCheck=collision_check, startRandom=True,numRestarts = 500)
    if ikflag == False:
        raise Exception("IK for start point failed!")
    qstart = robot.getConfig()
    print('Feasible ik start ', robot.getConfig())
    # then IK for goal
    qtmp = qstart[:]
    qtmp[1] += np.random.random() * np.pi + np.pi / 2  # rotation angle, pi/2 to 3pi/2
    robot.setConfig(qtmp)
    eepos = np.array(end_effector.getWorldPosition(null))
    rand_pos = eepos.copy()
    rand_pos[:2] += np.random.random(2) * 0.3 - 0.15
    rand_pos[2] += (np.random.random() * 0.8 - 0.4)  # this might be too large, we will see
    goal = rand_pos.tolist()
    objective = ik.objective(end_effector,local = [[0,0,0], unit_vec1],world =[goal, vo.madd(goal, unit_vec2, 1)])
    ikflag = ik.solve_global(objective, feasibilityCheck=collision_check, startRandom=True,numRestarts = 500)
    if ikflag == False:
        raise Exception("IK for goal point failed!")
    print("Feasible ik goal ", robot.getConfig())
    qgoal = robot.getConfig()
    return np.array(qstart), np.array(qgoal)


if __name__ == '__main__':
    main()
