import klampt
from klampt import vis
from klampt.math import so3,se3,vectorops
from klampt.model import sensing
import os
import numpy as np
import math
from PIL import Image

KINEMATIC_SIMULATE = False       #whether to run the full physics simulation or just the camera simulation
SAVE_IMAGES = True
SAVE_POINT_CLOUDS = True

world = klampt.WorldModel()
world.readFile("../../../data/simulation_test_worlds/sensortest.xml")
#world.readFile("../../../data/manipulation_worlds/tx90scenario0.xml")
robot = world.robot(0)

#vis.add("world",world)

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

print("===================================================================")
print("                      simple_camera.py                           ")
print("Press space repeatedly to generate some interesting robot movement")
print("Press Ctrl+C to exit")
if SAVE_IMAGES or SAVE_POINT_CLOUDS:
    print()
    print("Saving images / point clouds to 'camera_test_output/' directory")
print("===================================================================")
os.makedirs("camera_test_output",exist_ok=True)

image_count = 0

def callback():
    #this is called repeatedly by the visualization loop
    #all appropriate GL contexts are set up and made active so that 
    #GPU-accelerated sensing can be performed here
    global sim,sensor,image_count
    dt = 1.0 / 30.0
    if not KINEMATIC_SIMULATE:
        #this uses the simulation to update the robot and the sensors
        sim.simulate(dt)
        sim.updateWorld()
    else:
        #just uses the world and kinematic simulation to update the sensor
        sensor.kinematicSimulate(world,dt)
    
    #read the sensor data as well as conversion to point cloud
    rgb,depth = sensing.camera_to_images(sensor)
    pc = sensing.camera_to_points(sensor,points_format='Geometry3D',all_points=True,color_format='rgb')

    #show in vis
    T = sensor.getTransformWorld()
    pc.setCurrentTransform(*T)
    vis.add("pointcloud",pc)

    #Perform I/O
    if SAVE_IMAGES or SAVE_POINT_CLOUDS:
        dmax = float(sensor.getSetting("zmax"))
        print("Saving to camera_test_output/rgb_%04d.png, camera_test_output/depth_%04d.png, camera_test_output/pc_%04d.pcd"%(image_count,image_count,image_count))
    if SAVE_IMAGES:
        if rgb is not None:
            Image.fromarray(rgb).save("camera_test_output/rgb_%04d.png"%(image_count,))
        if depth is not None:
            #rescale to 0-255
            depth = (depth/dmax*255).astype(np.uint8)
            Image.fromarray(depth).save("camera_test_output/depth_%04d.png"%(image_count,))
    if SAVE_POINT_CLOUDS:
        pc.saveFile("camera_test_output/pc_%04d.pcd"%(image_count,))
    image_count += 1

#Need to set up OpenGL first before using GPU accelerated sensor simulation.
#GLEW doesn't work outside of the main thread, hence the use of vis.loop().
vis.loop(callback=callback)
vis.kill()

