# Code to reproduce issues with PointCloud setPointsAndProperties

import klampt
from klampt import *
import numpy as np
import time

print("KLAMPT VERSION:", klampt.__version__)
print("NUMPY VERSION:", np.__version__)

def testfunc(stl, point_list):
	pass

w = WorldModel()

sphere = GeometricPrimitive()
sphere.setSphere([1,0,0], 0.5)
vis.add('sphere', sphere, color=(1,0,0,1))

point_list = [[1,2,3],[2,3,4],[3,4,5]]
point_list2 = [[1,1,1],[2,2,2],[3,3,3]]

# setting one point cloud allows visualization, but closing the window results in a segmentation fault.
pc = PointCloud()
klampt_pc = pc.setPointsAndProperties(np.array(point_list,dtype=float))

# Setting a second point cloud causes segmentation fault before vis.show().
pc2 = PointCloud()
klampt_pc2 = pc2.setPointsAndProperties(np.array(point_list2,dtype=float))

pc3 = PointCloud()
klampt_pc3 = pc3.setPointsAndProperties(np.hstack((np.array(point_list,dtype=float),np.array(point_list2,dtype=float))))

# Doing anything with the data after calling setPointsAndProperties causes segmentation faults.
print(np.asarray(point_list,dtype=float))
print(np.asarray(point_list2,dtype=float))

vis.show()

while vis.shown():
    # using pass here instead of calling testfunc allows visualization to show before segmentation fault
    # pass
    testfunc(sphere, point_list)
    time.sleep(0.1)
vis.kill()