from klampt import *
from klampt.io.numpy_convert import *
from klampt.model.trajectory import Trajectory,RobotTrajectory
from klampt.io import resource

print("################### TESTING CONVERSIONS W / NUMPY ####################")

w = WorldModel()
w.loadFile("../../data/athlete_fractal_1.xml")

geom = w.terrain(0).geometry()
mesh = geom.getTriangleMesh()
pc = geom.convert('PointCloud').getPointCloud()

print("MESH CONVERT") 
mnp = to_numpy(mesh)
print("Numpy size",mnp[0].shape,mnp[1].shape)
m2 = from_numpy(mnp,'TriangleMesh')
print(len(m2.vertices),len(m2.indices))

print("POINT CLOUD CONVERT") 
pcnp = to_numpy(pc)
print("Numpy size",pcnp.shape)
pc2 = from_numpy(pcnp,'PointCloud')
print(pc2.numPoints(),pc2.numProperties())

print("GEOMETRY CONVERT") 
gnp = to_numpy(geom)
print("Numpy geometry has transform",gnp[0])

print("TRAJECTORY CONVERT")
resource.setDirectory('.')
traj = resource.get("../../data/motions/athlete_flex_opt.path")
trajnp = to_numpy(traj)
rtraj = RobotTrajectory(w.robot(0),traj.times,traj.milestones)
rtrajnp = to_numpy(rtraj)
rtraj2 = from_numpy(rtrajnp,template=rtraj)
print("Return type",rtraj2.__class__.__name__,"should be RobotTrajectory")

geom2 = w.robot(0).link(11).geometry()
vg = geom2.convert('VolumeGrid',0.01)
print("VOLUME GRID CONVERT")
vgnp = to_numpy(vg.getVolumeGrid())
print(vgnp[0],vgnp[1])

import matplotlib.pyplot as plt
import numpy as np
grid = vgnp[2]
gridslice = grid.shape[0]//2
vmin,vmax = np.min(grid[gridslice,:,:]),np.max(grid[gridslice,:,:])
print("Range:",vmin,vmax)
#plt.figure()
#plt.imshow(grid[gridslice,:,:])
#plt.show()

print("################### TESTING CONVERSIONS W / OPEN3D #####################")

try:
    from klampt.io.open3d_convert import *
    import open3d
except ImportError:
    print("It looks like open3D isn't installed, can't run this test")
    raise 

omesh = to_open3d(geom2.getTriangleMesh())
omesh.compute_vertex_normals()
ogrid = to_open3d(vg.getVolumeGrid())
open3d.visualization.draw_geometries([omesh,ogrid])

vmesh = from_open3d(omesh)
vgrid = from_open3d(ogrid)

from klampt import vis
vis.add("grid",Geometry3D(vgrid))
vis.setColor("grid",0,1,0)
vis.add("mesh",Geometry3D(vmesh))
vis.run()

opc = to_open3d(pc)
vpc = from_open3d(opc)

open3d.geometry.estimate_normals(opc, search_param = open3d.geometry.KDTreeSearchParamHybrid(
        radius = 0.25, max_nn = 30))
open3d.visualization.draw_geometries([opc])
