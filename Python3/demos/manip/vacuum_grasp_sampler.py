from klampt.manip.grasp_space import GraspParameterSpace
from klampt.manip.vaccum_flat_grasp_sampler import FlatAreaGraspSampler
from klampt.model.gripperinfo import GripperInfo
from klampt.math import se3,so3
from klampt import WorldModel
from klampt import vis
import numpy as np

if __name__ == '__main__':
    gripper = GripperInfo.load('../../../robotinfo/gripperinfo/robotiq_epick.json')
    sampler = FlatAreaGraspSampler(gripper, 100.0, 0.1)
    world = WorldModel()
    obj = world.makeRigidObject('env')
    res = obj.geometry().loadFile('../../../data/terrains/sunrgbdv2.json')
    extrinsics_file = '../../../data/terrains/sunrgbdv2_data/sunrgbdv2_extrinsics.txt'
    extrinsics = np.loadtxt(extrinsics_file)
    pose = so3.from_ndarray(extrinsics[:3,:3]),extrinsics[:3,3]
    #add an extra y-z flip
    pose = se3.mul((so3.rotation([1,0,0],-np.pi/2),[0]*3),pose)

    assert res
    obj.setTransform(*pose)
    obj.geometry().set(obj.geometry().convert('PointCloud'))
    sampler.init(world, obj)
    print("Best score",sampler.score())
    vis.add("origin",se3.identity(),hide_label=True,fancy=True,length=1.0)
    sampler.visDebug()
    vis.loop()
