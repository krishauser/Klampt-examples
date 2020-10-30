from klampt import *
from klampt.io import ros as kros
from klampt import vis
import rospy
import tf

TEST_TF_LISTENER = True

w = WorldModel()
w.loadFile("../../data/athlete_plane.xml")

rospy.init_node("klampt_ros_subscriber")
if not TEST_TF_LISTENER:
    js_sub = kros.object_subscriber("js",w.robot(0),queue_size=10)

listener = tf.TransformListener()

def onmesh(mesh):
    vis.add("terrain",Geometry3D(mesh))
    vis.setColor("terrain",0.5,0.2,0.1)
    print("Got a ROS Mesh message")
terrain_sub = kros.subscriber("terrain",'TriangleMesh',onmesh,queue_size=10)

def onpc(pc):
    vis.add("terrain_pc",Geometry3D(pc))
    vis.setColor("terrain_pc",1,0.5,0)
    vis.setAttribute("terrain_pc","size",5.0)

pc_sub = kros.subscriber("terrain_point_cloud",'PointCloud',onpc,queue_size=10)

from sensor_msgs.msg import CameraInfo
def oncamerainfo(ci):
    vp = vis.getViewport()
    kros.from_CameraInfo(ci,vp)
    vis.setViewport(vp)
camera_sub = rospy.Subscriber('camera_info',CameraInfo,oncamerainfo,queue_size=10)

visworld = w.copy()
vis.add("world",visworld)
vis.show()
while not rospy.is_shutdown() and vis.shown():
    if TEST_TF_LISTENER:
        kros.listen_tf(listener,w)
        vis.lock()
        for i in range(w.robot(0).numLinks()):
            visworld.robot(0).link(i).setTransform(*w.robot(0).link(i).getTransform())
        vis.unlock()
    else:
        vis.lock()
        visworld.robot(0).setConfig(w.robot(0).getConfig())
        vis.unlock()
    rospy.sleep(0.01)

