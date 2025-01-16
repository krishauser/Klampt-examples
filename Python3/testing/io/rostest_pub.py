from klampt import *
from klampt.io import ros as kros
from klampt import vis
import rospy
import tf

w = WorldModel()
w.loadFile("../../data/athlete_fractal_1.xml")
#you can configure the JointState message with its arguments
print(kros.to_JointState(w.robot(0)))
print(kros.to_JointState(w.robot(0),dq=None))

rospy.init_node("klampt_ros_publisher")
broadcaster = tf.TransformBroadcaster()
kros.broadcast_tf(broadcaster,w)

mesh = w.terrain(0).geometry().getTriangleMesh()
terrain_pub = kros.object_publisher("terrain",mesh,latch=True,queue_size=10)
terrain_pub.publish(mesh)

pc = w.terrain(0).geometry().convert('PointCloud').getPointCloud()
pc_pub = kros.object_publisher("terrain_point_cloud",pc,latch=True,queue_size=10)
pc_pub.publish(pc)

vis.add("world",w)
vis.show()

camera_pub = kros.object_publisher('camera_info',vis.getViewport(),latch=True,queue_size=10)
camera_pub.publish(vis.getViewport())

sim = Simulator(w)
js_pub = kros.object_publisher("js",sim.controller(0),convert_kwargs={'q':'actual','dq':'actual','effort':'actual'},queue_size=10)

print("Simulating and publishing...")
while not rospy.is_shutdown() and vis.shown():
    sim.simulate(0.01)

    kros.broadcast_tf(broadcaster,w)
    js_pub.publish(sim.controller(0))
    camera_pub.publish(vis.getViewport())
    
    

