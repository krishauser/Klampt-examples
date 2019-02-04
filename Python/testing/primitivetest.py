from klampt.model.create import *
from klampt import vis

b = box(0.1,0.5,1.0,center=(1,0,0),type='GeometricPrimitive')
s = sphere(0.4,center=(0,-1,0),type='GeometricPrimitive')
b2 = box(0.1,0.5,1.0,center=(1,0,0),type='TriangleMesh')
s2 = sphere(0.4,center=(0,-1,0),type='TriangleMesh')


vis.add("box",b)
vis.add("sphere",s)
vis.add("box_mesh",b2)
vis.add("sphere_mesh",s2)
vis.setColor("box",1,0,0,0.5)
vis.setColor("sphere",0,1,0,0.5)
vis.setColor("box_mesh",1,0,0)
vis.setColor("sphere_mesh",0,1,0)

vis.show()
vis.spin(float('inf'))
vis.kill()
