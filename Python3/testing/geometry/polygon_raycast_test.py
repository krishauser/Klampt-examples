from klampt import *
from klampt.math import so3,vectorops

#Testing polygon - ray intersection
polygon = GeometricPrimitive()
polygon.setPolygon([0,0,0,  1,0,0,  0.5,0.5,0, 1,1,0,  0,1,0])
c = Geometry3D(polygon)
c.setCurrentTransform(so3.identity(),[0,0,-0.5])
vis.add("polygon",c,color=[0.2,0,0.6,1])
for i in range(10):
    x = i*0.2 - 0.2
    y = 0.5
    z = 1
    s,d = [x,y,z],vectorops.unit([0,-0.2,-1])
    elem,pt = c.rayCast_ext(s,d)
    vis.add("ray "+str(i),[s,vectorops.madd(s,d,2)],color=(1,1,0,1))
    if elem >= 0:
        vis.add("polygon hit "+str(i),pt,color=[0.4,0,1,1])

vis.loop()