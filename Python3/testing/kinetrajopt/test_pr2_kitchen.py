"""
Test the example of moving the pr2 around a table
This one has more environment obstacles and is more complicated
"""
import sys
import copy
import numpy as np

from xml.dom import minidom
from klampt import WorldModel, vis
from klampt.model import create, trajectory

from klampt.plan.kinetrajopt import KineTrajOpt, TrajOptSettings


def main():
    world, robot, link_index, geom_index, obs_index = create_world()
    # get start and target for the joints
    joint_start = [ 0.115 ,  0.6648, -0.3526, 1.6763, -1.9242, 2.9209,  -1.2166,
                    1.3425, -0.6365,  0.0981, -1.226, -2.0264, -3.0125, -1.3958,
                    -1.9289,  2.9295,  0.5748, -3.137]
    joint_target = [0.115,  0.6808, -0.3535, 1.4343, -1.8516, 2.7542, -1.2005,
                    1.5994, -0.6929, -0.3338, -1.292, -1.9048, -2.6915, -1.2908,
                    -1.7152,  1.3155,  0.6877, -0.0041]
    assert(len(link_index) == len(joint_start) == len(joint_target))
    # generate guess
    n_grid = 10  # resolution
    guess = np.linspace(joint_start, joint_target, n_grid)
    # create trajopt instance
    config = TrajOptSettings(cvxpy_args={'solver': 'GUROBI'}, limit_joint=False)
    trajopt = KineTrajOpt(world, robot, joint_start, joint_target, config=config, link_index=link_index,
                          geom_index=geom_index, obs_index=obs_index)
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
    dists = []
    for i in range(1, n_grid - 1):  # set to -2 so the robot stops there with collision
        vis.add('ghost%d' % i, list(get_config(sol[i])))
        vis.setColor('ghost%d' % i, 0, 1, 0, 0.3)
        robot.setConfig(get_config(sol[i]))
    update_config(sol[-1])
    robot.setConfig(config)
    vis.spin(float('inf'))


def compute_distance(world, robot, link_index, obs_index):
    dists = []
    for obsi in obs_index:
        obs_geom = world.terrain(obsi).geometry().convert('ConvexHull', 0)
        for linki in link_index:
            try:
                link_geom = robot.link(linki).geometry().convert('ConvexHull', 0)
            except:
                continue
            # get and set transformation
            tran = robot.link(linki).getTransform()
            link_geom.setCurrentTransform(*tran)
            dists.append(obs_geom.distance(link_geom).d)
    return dists


def create_world():
    """Just create the world so geometries are clearly defined"""
    load_path = './assets/kitchen.env.xml'
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
    link_names = [
        'torso_lift_link',
        'l_shoulder_pan_link', 'l_shoulder_lift_link', 'l_upper_arm_roll_link', 
        'l_elbow_flex_link', 'l_forearm_roll_link', 'l_wrist_flex_link', 'l_wrist_roll_link',
        'r_shoulder_pan_link', 'r_shoulder_lift_link', 'r_upper_arm_roll_link',
        'r_elbow_flex_link', 'r_forearm_roll_link', 'r_wrist_flex_link', 'r_wrist_roll_link',
        'base0', 'base1', 'base3'
    ]
    geom_names = ['base_link',
        'torso_lift_link',
        'l_shoulder_pan_link', 'l_shoulder_lift_link', 'l_upper_arm_roll_link', 
        'l_elbow_flex_link', 'l_forearm_roll_link', 'l_wrist_flex_link', 'l_wrist_roll_link',
        'r_shoulder_pan_link', 'r_shoulder_lift_link', 'r_upper_arm_roll_link',
        'r_elbow_flex_link', 'r_forearm_roll_link', 'r_wrist_flex_link', 'r_wrist_roll_link']
    link_indexes = []
    geom_indexes = []
    for link_name in link_names:
        link = robot.link(link_name)
        link_indexes.append(link.getIndex())
    for link_name in geom_names:
        link = robot.link(link_name)
        geom_indexes.append(link.getIndex())
    obs_indexes = [i for i in range(world.numTerrains())][-3::]
    return world, robot, link_indexes, geom_indexes, obs_indexes


if __name__ == "__main__":
    main()
