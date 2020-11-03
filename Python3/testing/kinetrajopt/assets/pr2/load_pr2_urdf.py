"""
Try to load the urdf of pr2.
"""
from klampt import WorldModel, vis

world = WorldModel()
world.readFile('pr2.urdf')
vis.add('world', world)
vis.spin(100)
