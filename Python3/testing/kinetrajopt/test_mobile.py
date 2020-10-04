"""
In this task, I show a mobile robot also works
"""
from xml.dom import minidom
import numpy as np

from klampt import WorldModel, vis
from klampt.model import create
from klampt.plan.kinetrajopt import KineTrajOpt, TrajOptSettings


def main():
    world, robot, link_index, geom_index, obs_index = create_world()
    q0 = [0, -2, 0]
    qf = [3, 1, 3.14]
    config = TrajOptSettings(cvxpy_args={'solver': 'GUROBI'}, limit_joint=False)
    trajopt = KineTrajOpt(world, robot, q0=q0, qf=qf, config=config, link_index=link_index, geom_index=geom_index, obs_index=obs_index)
    # create guess and solve
    n_grid = 10
    guess = np.linspace(q0, qf, n_grid)
    rst = trajopt.optimize(guess)
    print(rst)
    traj = rst['sol']
    # show the results
    vis.add('world', world)
    robot.setConfig(traj[-1])
    for i, qi in enumerate(traj[:-1]):
        name = 'ghost%d' % i
        vis.add(name, qi)
        vis.setAttribute(name, 'type', 'Config')
        vis.setColor(name, 0, 1, 0, 0.4)
    vis.spin(100)


def create_world():
    world = WorldModel()
    # world.loadElement('robots_urdf/base_link.off')
    world.readFile('./assets/cardata/car.rob')
    robot = world.robot(0)
    robot.setConfig([0, -2, 0])
    link = robot.link(2)
    link_geom = link.geometry()
    link_geom.scale(4)  # make the geometry larger so it's more difficult
    # world.readFile('./robots_urdf/05-visual.urdf')
    # load the environment too
    load_path = './assets/jagged_narrow.xml'
    xmldoc = minidom.parse(load_path)
    itemlist = xmldoc.getElementsByTagName('Geom')
    for i, item in enumerate(itemlist):
        extends = [float(x) * 2 for x in item.getElementsByTagName('extents')[0].childNodes[0].data.split()]
        translates = [float(x) for x in item.getElementsByTagName('translation')[0].childNodes[0].data.split()]
        create.primitives.box(extends[0], extends[1], extends[2], translates, world=world, name='obstacle%d' % i)
    return world, robot, [0, 1, 2], [2], [0, 1, 2, 3, 4]  # the last three are link_index, geom_index, obs_index


if __name__ == "__main__":
    main()
