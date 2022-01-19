import klampt
import math
from klampt import vis, Appearance
from klampt.math import so3,se3,vectorops
from klampt.vis.glinterface import GLPluginInterface
from klampt.model import sensing
import math
import time
import random
import math

try:
	import matplotlib.pyplot as plt
	HAVE_PYPLOT = True
except ImportError:
	HAVE_PYPLOT = False
	print("**** Matplotlib not available, can't plot color/depth images ***")


KINEMATIC_SIMULATE = False       #whether to run the full physics simulation or just the camera simulation
MANUAL_DEPTH_PROCESSING = False  #whether to use the convenience routines (recommended) or not
GET_POINTS_FROM_IMAGES = False   #whether to get the rgb/depth images first, then use Klampt utilities to get the point cloud
READ_POINTS_LOCAL = True         #whether to transform the point cloud to world coordinates

def processDepthSensor(sensor):
	if MANUAL_DEPTH_PROCESSING:
		#manual processing
		data = sensor.getMeasurements()
		w = int(sensor.getSetting("xres"))
		h = int(sensor.getSetting("yres"))
		#first, RGB then depth
		mind,maxd = float('inf'),float('-inf')
		for i in range(h):
			for j in range(w):
				pixelofs = (j+i*w)
				rgb = int(data[pixelofs])
				depth = data[pixelofs+w*h]
				mind = min(depth,mind)
				maxd = max(depth,maxd)
		print("Depth range",mind,maxd)
	else:
		rgb,depth = sensing.camera_to_images(sensor)
		return rgb,depth

world = klampt.WorldModel()
world.readFile("../../data/simulation_test_worlds/sensortest.xml")
#world.readFile("../../data/tx90scenario0.xml")
robot = world.robot(0)

vis.add("world",world)

sim = klampt.Simulator(world)
sensor = sim.controller(0).sensor("rgbd_camera")
print("sensor.getSetting('link'):",sensor.getSetting("link"))
print("sensor.getSetting('Tsensor'):",sensor.getSetting("Tsensor"))
input("Press enter to continue...")

#The next lines will set up an external camera by modifying the SimRobotSensor's properties
#T = (so3.sample(),[0,0,1.0])
T = (so3.mul(so3.rotation([1,0,0],math.radians(-10)),[1,0,0, 0,0,-1,  0,1,0]),[0,-2.0,0.5])
sensing.set_sensor_xform(sensor,T,link=-1)

vis.add("sensor",sensor)


#Note: GLEW sensor simulation only runs if it occurs in the visualization thread (e.g., the idle loop)
class SensorTestWorld (GLPluginInterface):
	def __init__(self):
		GLPluginInterface.__init__(self)
		robot.randomizeConfig()
		#sensor.kinematicSimulate(world,0.01)
		sim.controller(0).setPIDCommand(robot.getConfig(),[0.0]*7)

		self.rgb=None
		self.depth=None
		self.compute_pc = False
		self.pc = None
		self.pc_appearance = None
		self.original_view = None

		def randomize():
			robot.randomizeConfig()
			sim.controller(0).setPIDCommand(robot.getConfig(),[0.0]*7)
		def plot_rgb():
			self.rgb,self.depth = processDepthSensor(sensor)
			if self.rgb is not None:
				plt.imshow(self.rgb)
				plt.show()
		def plot_depth():
			self.rgb,self.depth = processDepthSensor(sensor)
			if self.depth is not None:
				plt.imshow(self.depth)
				plt.show()
		def toggle_point_cloud():
			self.compute_pc = not self.compute_pc
		def save_point_cloud():
			if self.pc != None:
				if isinstance(self.pc,klampt.Geometry3D):
					print("Saving to sensortest_temp.pcd")
					self.pc.saveFile("sensortest_temp.pcd")
				else:
					print("Saving to sensortest_temp.csv")
					import numpy
					numpy.savetxt("sensortest_temp.csv",self.pc)
				input("Saved...")
		def toggle_view():
			if self.original_view is None:
				self.original_view = self.view
				v = sensing.camera_to_viewport(sensor,robot)
				self.window.program.set_view(v)
				self.view = v
			else:
				self.window.program.set_view(self.original_view)
				self.view = self.original_view
				self.original_view = None
		def print_view():
			print("Tgt",self.view.camera.tgt)
			print("Rot",self.view.camera.rot)

		self.add_action(randomize,'Randomize configuration',' ')
		if HAVE_PYPLOT:
			self.add_action(plot_rgb,'Plot color','c')
			self.add_action(plot_depth,'Plot depth','d')
		self.add_action(toggle_point_cloud,'Toggle point cloud drawing','p')
		self.add_action(save_point_cloud,'Save point cloud','s')
		self.add_action(toggle_view,'Toggle views','v')
		self.add_action(print_view,'Print current view','p')

	def idle(self):
		#print "Idle..."
		if not KINEMATIC_SIMULATE:
			#this uses the simulation to update the robot and the sensors
			sim.simulate(0.01)
			sim.updateWorld()
		else:
			#just uses the world and kinematic simulation to update the sensor
			sensor.kinematicSimulate(world,0.01)
		try:
			#self.rgb,self.depth = processDepthSensor(sensor)
			if self.compute_pc:
				t0 = time.time()
				if GET_POINTS_FROM_IMAGES:
					#manual processing
					self.rgb,self.depth = processDepthSensor(sensor)
					self.rgb[:,:,0] = 0
					self.pc = sensing.image_to_points(self.depth,self.rgb,float(sensor.getSetting('xfov')),float(sensor.getSetting('yfov')),points_format='Geometry3D',all_points=True)
					self.pc.setCurrentTransform(*T)
					self.pc_appearance = Appearance()
				else:
					if READ_POINTS_LOCAL:
						#local point cloud
						#self.pc = sensing.camera_to_points(sensor,points_format='numpy',color_format='rgb')
						#self.pc = sensing.camera_to_points(sensor,points_format='native')
						self.pc = sensing.camera_to_points(sensor,points_format='Geometry3D',all_points=True,color_format='rgb')
						self.pc.setCurrentTransform(*T)
						self.pc_appearance = Appearance()
					else:
						#world point cloud
						#self.pc = sensing.camera_to_points_world(sensor,robot,points_format='numpy')
						self.pc = sensing.camera_to_points_world(sensor,robot,points_format='Geometry3D',color_format='rgb')
						self.pc_appearance = Appearance()
				#print("Read and process PC time",time.time()-t0)
		except Exception as e:
			print(e)
			import traceback
			traceback.print_exc()
			exit(-1)
		return True

	def display(self):
		GLPluginInterface.display(self)
		if self.pc is not None and self.compute_pc:
			#manual drawing of native or numpy point clouds
			t0 = time.time()
			if isinstance(self.pc,klampt.Geometry3D):
				#Geometry3D drawing
				self.pc_appearance.drawWorldGL(self.pc)
			else:
				#manual OpenGL drawing
				glDisable(GL_LIGHTING)
				glPointSize(5.0)
				glColor3f(0,0,0)
				glBegin(GL_POINTS)
				for pt in self.pc:
					if len(pt) == 6:
						glColor3f(*pt[3:6])
					if READ_POINTS_LOCAL:
						glVertex3fv(se3.apply(T,pt[0:3]))
					else:
						glVertex3fv(pt[0:3])
				glEnd()
			#print "Draw PC time",time.time()-t0
		return True

vis.pushPlugin(SensorTestWorld())
vis.spin(float('inf'))
vis.kill()

"""
#Note: GLEW doesn't work outside of the main thread, hence the use of the GLPluginInterface. 
#The below code falls back to the non-accelerated sensor simulation

vis.show()
time.sleep(0.5)

for sample in range(10):
	vis.lock()
	robot.randomizeConfig()
	#sensor.kinematicSimulate(world,0.01)
	sim.controller(0).setPIDCommand(robot.getConfig(),[0.0]*7)
	vis.unlock()
	for i in range(100):
		sim.simulate(0.01)
		vis.lock()
		sim.updateWorld()
		vis.unlock()
		time.sleep(0.002)

	processDepthSensor()
	raw_input()

vis.hide()
vis.kill()
"""

