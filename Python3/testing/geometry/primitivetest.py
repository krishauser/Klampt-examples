from klampt.model.create import *
from klampt import vis
from klampt.math import so3
import math

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

prim1 = GeometricPrimitive()
prim1.setTriangle([0,0,1],[0.5,0,1],[0,0.5,1])
vis.add("prim1",prim1,color=[0.5,0.5,1])
prim2 = GeometricPrimitive()
prim2.setPolygon([0,0,1.2]+[0.5,0,1.2]+[0.5,0.5,1.2]+[0,0.5,1.2])
vis.add("prim2",prim2,color=[0.5,1,0.5])
prim3 = GeometricPrimitive()
prim3.setBox([0,0,1.5],so3.rotation([0,1,0],math.radians(10)),[0.5,0.5,0.5])
vis.add("prim3",prim3,color=[1,0.5,0.5])
seg = GeometricPrimitive()
seg.setSegment([-0.75,0,0],[0.75,0,0])
vis.add("prim4",seg,color=[0,1.0,0.5])

vis.loop()
