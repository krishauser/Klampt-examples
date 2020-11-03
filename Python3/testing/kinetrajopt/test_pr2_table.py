"""
Test the example of moving the right arm of pr2 under table.
Joschu has a ridiculously fast algorithm...
"""
import sys
import copy
import numpy as np
import os

from xml.dom import minidom
from klampt import WorldModel, vis
from klampt.model import create, trajectory

from klampt.plan.kinetrajopt import KineTrajOpt, TrajOptSettings


def main():
    world, robot, link_index, geom_index, obs_index = create_world()
    joint_start = [-1.832, -0.332, -1.011, -1.437, -1.1, -1.926,  3.074]
    joint_target = [0.062, 1.287, 0.1, -1.554, -3.011, -0.268, 2.988]
    # generate guess
    n_grid = 10  # resolution
    guess = np.linspace(joint_start, joint_target, n_grid)
    # create trajopt instance
    config = TrajOptSettings(cvxpy_args={'solver': 'GUROBI'}, limit_joint=False)
    trajopt = KineTrajOpt(world, robot, joint_start, joint_target, config=config, link_index=link_index, geom_index=geom_index, obs_index=obs_index)
    # create some stuff for visualization
    config = np.array(robot.getConfig())

    def update_config(q):
        config[link_index] = q

    def get_config(q):
        cq = copy.copy(config)
        cq[link_index] = q
        return cq

    rst = trajopt.optimize(guess)
    sol = rst['sol']
    vis.add('world', world)
    for i in range(1, n_grid - 2):  # set to -2 so the robot stops there with collision
        vis.add('ghost%d' % i, list(get_config(sol[i])))
        vis.setColor('ghost%d' % i, 0, 1, 0, 0.5)
        # dists = compute_distance(world, robot, link_index, obs_index)
        # print(dists)
    update_config(sol[-1])
    vis.spin(float('inf'))


def compute_distance(world, robot, link_index, obs_index):
    dists = []
    for obsi in obs_index:
        obs_geom = world.terrain(obsi).geometry().convert('ConvexHull', 0)
        for linki in link_index:
            link_geom = robot.link(linki).geometry().convert('ConvexHull', 0)
            # get and set transformation
            tran = robot.link(linki).getTransform()
            link_geom.setCurrentTransform(*tran)
            dists.append(obs_geom.distance(link_geom).d)
    return dists


def create_world():
    """Just create the world so geometries are clearly defined"""
    load_path = './assets/table.xml'
    xmldoc = minidom.parse(load_path)
    itemlist = xmldoc.getElementsByTagName('Geom')
    print('num body = ', len(itemlist))
    world = WorldModel()
    for i, item in enumerate(itemlist):
        extends = [float(x) * 2 for x in item.getElementsByTagName('extents')[0].childNodes[0].data.split()]
        translates = [float(x) for x in item.getElementsByTagName('translation')[0].childNodes[0].data.split()]
        create.primitives.box(extends[0], extends[1], extends[2], translates, world=world, name='obstacle%d' % i)
    # load the robot
    pr2_path = './assets/pr2/pr2.urdf'
    world.readFile(pr2_path)
    robot = world.robot(0)
    print(f'It has {robot.numLinks()} links')
    # link_names = ['r_shoulder_pan_link', 'r_shoulder_lift_link', 'r_upper_arm_roll_link', 'r_elbow_flex_link', 'r_forearm_roll_link', 'r_wrist_flex_link', 'r_wrist_roll_link']
    link_names = ['r_shoulder_pan_link', 'r_shoulder_lift_link', 'r_upper_arm_roll_link',
        'r_elbow_flex_link', 'r_forearm_roll_link', 'r_wrist_flex_link', 'r_wrist_roll_link',]
    geom_names = ['r_shoulder_pan_link', 'r_shoulder_lift_link', 'r_upper_arm_roll_link', 'r_upper_arm_link',
        'r_elbow_flex_link', 'r_forearm_roll_link', 'r_forearm_link', 'r_wrist_flex_link', 'r_wrist_roll_link',]
    link_indexes = []
    for link_name in link_names:
        link = robot.link(link_name)
        link_indexes.append(link.getIndex())
    geom_indexes = []
    for link_name in geom_names:
        link = robot.link(link_name)
        geom_indexes.append(link.getIndex())
    obs_indexes = [0]
    return world, robot, link_indexes, geom_indexes, obs_indexes


if __name__ == "__main__":
    main()
