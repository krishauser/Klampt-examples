from klampt.model.create import *
from klampt import Geometry3D,Mass
import sys

if len(sys.argv) > 1:
    a = Geometry3D()
    if not a.loadFile(sys.argv[1]):
        print("Error loading",sys.argv[1])
        exit()
else:
    #a = box(0.1,0.5,1.0,center=(0,0,0),type='GeometricPrimitive')
    a = sphere(0.5,center=(0,0,0),type='GeometricPrimitive')
if len(sys.argv) > 2:
    b = Geometry3D()
    if not b.loadFile(sys.argv[2]):
        print("Error loading",sys.argv[2])
        exit()
else:
    b = sphere(0.4,center=(0,0,0),type='GeometricPrimitive')
    gp = b.getGeometricPrimitive() # type: GeometricPrimitive

    #seg = GeometricPrimitive()
    #seg.setSegment([-0.25,0,0],[0.25,0,0])
    #b = Geometry3D(seg)

#Klampt >= 0.10
geomtypes = ['GeometricPrimitive','TriangleMesh','PointCloud','ImplicitSurface','OccupancyGrid','ConvexHull','Heightmap']
#geomtypes = ['GeometricPrimitive','TriangleMesh','PointCloud','ImplicitSurface','OccupancyGrid','Heightmap']
#Klampt < 0.10
#geomtypes = ['GeometricPrimitive','TriangleMesh','PointCloud','VolumeGrid']
tparams = {'PointCloud':0.02,'VolumeGrid':0.04,'ImplicitSurface':0.04,'OccupancyGrid':0.04,'Heightmap':0.01}
atypes = dict()
btypes = dict()
for t in geomtypes:
    atypes[t] = a
    btypes[t] = b
    try:
        if a.type() != t:
            at = a.convert(t,tparams.get(t,0))
            atypes[t] = at
    except Exception as e:
        print("Could not convert",a.type(),"to",t)
        raise
    try:
        if b.type() != t:
            bt = b.convert(t,tparams.get(t,0))
            btypes[t] = bt
    except Exception as e:
        print("Could not convert",b.type(),"to",t)
        raise

for (t,g) in atypes.items():
    m = Mass()
    m.estimate(g,1.0,1.0)
    print(t)
    print("  COM surface-only %.3f %.3f %.3f"%tuple(m.getCom()))
    H = m.getInertia()
    print("  Inertia surface-only %.3f %.3f %.3f"%(H[0],H[4],H[8]))
    m.estimate(g,1.0,0.0)
    H = m.getInertia()
    print("  COM volume-only %.3f %.3f %.3f"%tuple(m.getCom()))
    print("  Inertia volume-only %.3f %.3f %.3f"%(H[0],H[4],H[8]))
