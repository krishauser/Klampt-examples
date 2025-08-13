from klampt.manip.grasp_space import GraspParameterSpace
from klampt.model.gripperinfo import GripperInfo
from klampt.math import se3
import random
from klampt import vis

if __name__ == '__main__':
    robotiq_85 = GripperInfo.load('../../../robotinfo/gripperinfo/robotiq_85.json')
    #items = [['ikConstraint','endPosition',0],['ikConstraint','endPosition',1],'fingerConfig']
    #items = ['transform']
    #items = [['transform',3],['transform',4],['transform',5]] #only translation
    #items = ['fingerDriverConfig']
    items = ['transform','fingerDriverConfig'] #transform and driver configuration

    space = GraspParameterSpace(robotiq_85,items)
    nd = space.numDims()
    space.setPositionBounds([-0.5]*3,[0.5]*3)
    print(space.toFeatures(space.template))
    print(space.featureBounds())
    print(space.toGrasp([0]*nd))
    print(space.toGrasp([1]*nd))

    vis.setWindowTitle("Sampled grasps")
    vis.add("origin",se3.identity(),hide_label=True,fancy=True,length=1.0)
    featureBounds = space.featureBounds()
    for i in range(100):
        f = [random.uniform(b[0], b[1]) for b in zip(*featureBounds)]
        g = space.toGrasp(f)
        # T = g.asFixedGrasp().ikConstraint.getTransform()
        # robotiq_85.addToVis(base_xform=T, prefix="grasp_%d" % i, hide_label=False
        g.addToVis("grasp_%d" % i, gripper=robotiq_85, hide_label=True)
    vis.loop()