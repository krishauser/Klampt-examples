import klampt
import math
from klampt import vis, Appearance
from klampt.math import so3,se3,vectorops
from klampt.vis.glinterface import GLPluginInterface
from klampt.model import sensing
from klampt import __version__
import math
import time
import random
import math

klampt_version = tuple([int(x) for x in __version__.split('.')])

try:
	import matplotlib.pyplot as plt
	HAVE_PYPLOT = True
except ImportError:
	HAVE_PYPLOT = False
	print("**** Matplotlib not available, can't plot color/depth images ***")


KINEMATIC_SIMULATE = False       #whether to run the full physics simulation or just the camera simulation


world = klampt.WorldModel()
world.readFile("../../../data/simulation_test_worlds/sensortest.xml")
#world.readFile("../../../data/manipulation_worlds/tx90scenario0.xml")
robot = world.robot(0)

vis.add("world",world)

sim = klampt.Simulator(world)
sensor = sim.controller(0).sensor("rgbd_camera")
print("sensor.link:",sensor.link.index)
print("sensor.transform:",sensor.getTransform())

#The next lines will change the model to an external camera by modifying the SensorModel's properties
#T = (so3.sample(),[0,0,1.0])
T = (so3.mul(so3.rotation([1,0,0],math.radians(-10)),[1,0,0, 0,0,-1,  0,1,0]),[0,-2.0,0.5])
sensing.set_sensor_xform(sensor,T,link=-1)
print()
print("Changed to external camera:")
print("sensor.link:",sensor.link)
print("sensor.transform:",sensor.getTransform())
print()

vis.add("sensor",sensor)

#set up a way to generate random configurations
def randomize():
	robot.randomizeConfig()
	sim.controller(0).setPIDCommand(robot.getConfig(),[0.0]*7)
randomize()
vis.addAction(randomize,'Randomize configuration',' ')

#some interactive stuff to play with cameras and point clouds
class SensorTestCallbacks:
	def __init__(self):
		self.compute_pc = False
		self.original_view = None
		self.pc = None

		def plot_rgb():
			rgb,depth = sensing.camera_to_images(sensor)
			if rgb is not None:
				plt.imshow(rgb)
				plt.show()
		def plot_depth():
			rgb,depth = sensing.camera_to_images(sensor)
			if depth is not None:
				plt.imshow(depth)
				plt.show()
		def toggle_point_cloud():
			self.compute_pc = not self.compute_pc
			if not self.compute_pc:
				try:
					vis.remove("pc")
				except Exception as e:
					pass
		def save_point_cloud():
			if self.pc != None:
				if isinstance(self.pc,klampt.Geometry3D):
					print("Saving to camera_test_output/point_cloud.pcd")
					self.pc.saveFile("camera_test_output/point_cloud.pcd")
				else:
					print("Saving to camera_test_output/point_cloud.csv")
					import numpy
					numpy.savetxt("camera_test_output/point_cloud.csv",self.pc)
				input("Saved...")
		def toggle_view():
			if self.original_view is None:
				self.original_view = vis.getViewport()
				v = sensing.camera_to_viewport(sensor,robot)
				vis.setViewport(v)
				vis.resizeWindow(v.w,v.h)
			else:
				vis.setViewport(self.original_view)
				vis.resizeWindow(self.original_view.w,self.original_view.h)
				self.original_view = None
		def print_view():
			view = vis.getViewport()
			if klampt_version >= (0,10,0):
				print("Tgt",view.controller.tgt)
				print("Rot",view.controller.rot)
			else:
				print("Tgt",view.camera.tgt)
				print("Rot",view.camera.rot)

		if HAVE_PYPLOT:
			vis.addAction(plot_rgb,'Plot color','c')
			vis.addAction(plot_depth,'Plot depth','d')
		vis.addAction(toggle_point_cloud,'Toggle point cloud drawing','p')
		vis.addAction(save_point_cloud,'Save point cloud','s')
		vis.addAction(toggle_view,'Toggle views','v')
		vis.addAction(print_view,'Print current view','p')

	def callback(self):
		try:
			if self.compute_pc:
				t0 = time.time()
				self.pc = sensing.camera_to_points(sensor,points_format='Geometry3D',all_points=True,color_format='rgb')
				T = sensor.getTransformWorld()
				self.pc.setCurrentTransform(*T)
				#print("Read and process PC time",time.time()-t0)
				vis.add("pc",self.pc)
		except Exception as e:
			print(e)
			import traceback
			traceback.print_exc()
			exit(-1)
		return True

cb = SensorTestCallbacks()
def callback():
	if not KINEMATIC_SIMULATE:
		#this uses the simulation to update the robot and the sensors
		sim.simulate(0.01)
		sim.updateWorld()
	else:
		#just uses the world and kinematic simulation to update the sensor
		sensor.kinematicSimulate(world,0.01)
	cb.callback()

vis.loop(callback=callback)
vis.kill()

