from klampt import *
from klampt.math import so3,se3,vectorops
from klampt.model import sensing
from klampt.io import numpy_convert
import math
import random
from typing import Tuple
import time

def set_up_world() -> Tuple[WorldModel,Simulator,SensorModel]:
    world = WorldModel()
    world.readFile("../../../data/simulation_test_worlds/sensortest.xml")
    robot = world.robot(0)

    sim = Simulator(world)
    sensor = sim.controller(0).sensor("rgbd_camera")
    #The next lines will change the model to an external camera 
    T = (so3.mul(so3.rotation([1,0,0],math.radians(-10)),[1,0,0, 0,0,-1,  0,1,0]),[0,-2.0,0.5])
    sensing.set_sensor_xform(sensor,T,link=-1)
    return world,sim,sensor

def run_sensing_tests(sensor: SensorModel):
    #normal processing
    pc = sensing.camera_to_points(sensor,points_format='Geometry3D',all_points=True,color_format='rgb')
    T = sensor.getTransformWorld()
    pc.setCurrentTransform(*T)
    vis.add("pc",pc)

    #manual processing
    rgb,depth = sensing.camera_to_images(sensor)
    rgb[:,:,0] = 0  #testing weird coloration
    pc = sensing.image_to_points(depth,rgb,xfov=float(sensor.getSetting('xfov')),yfov=float(sensor.getSetting('yfov')),points_format='Geometry3D',all_points=True)
    pc.setCurrentTransform(*se3.mul((so3.identity(),[0,0,-2]),T))
    vis.add("pc_manual",pc)

    #world point cloud
    pc = sensing.camera_to_points_world(sensor,points_format='Geometry3D',color_format='rgb')
    pc.setCurrentTransform(*se3.mul((so3.identity(),[0,0,-4]),pc.getCurrentTransform()))
    vis.add("pc_world",pc)

    # #local point cloud
    pc_numpy = sensing.camera_to_points(sensor,points_format='numpy',color_format='rgb')
    # pc = numpy_convert.from_numpy(pc_numpy,'PointCloud')

    # #world point cloud
    pc_numpy = sensing.camera_to_points_world(sensor,points_format='numpy',color_format='rgb')
    # pc = numpy_convert.from_numpy(pc_numpy,'PointCloud')

    #triangle mesh form
    pc,app = sensing.camera_to_points(sensor,points_format='TriangleMesh',all_points=True)
    pc = Geometry3D(pc)
    pc.setCurrentTransform(*se3.mul((so3.identity(),[0,0,-6]),T))
    app.setSilhouette(0.0)  #improves speed
    vis.add("pc_mesh",pc,appearance=app)

    #heightmap form
    pc = sensing.camera_to_points(sensor,points_format='Heightmap')
    pc = Geometry3D(pc)
    pc.setCurrentTransform(*se3.mul((so3.identity(),[0,0,-8]),T))
    vis.add("pc_heightmap",pc)


def test_sim_simulate(duration = float('inf')):
    world,sim,sensor = set_up_world()
    #vis.add("world",world)
    vis.add("sensor",sensor)

    def callback():
        world.robot(0).randomizeConfig()
        sim.controller(0).setPIDCommand(world.robot(0).getConfig(),[0.0]*7)
        sim.simulate(0.01)
        sim.updateWorld()
        run_sensing_tests(sensor,world.robot(0))
        if vis.animationTime() > duration:
            vis.show(False)

    vis.loop(callback=callback)
    vis.kill()

def test_kinematic_simulate(duration = float('inf')):
    world,sim,sensor = set_up_world()
    #vis.add("world",world)
    vis.add("sensor",sensor)

    def callback():
        world.robot(0).randomizeConfig()
        sensor.kinematicSimulate(world,0.01)
        run_sensing_tests(sensor,world.robot(0))
        if vis.animationTime() > duration:
            vis.show(False)

    vis.loop(callback=callback)
    vis.kill()

def test_multithreaded(duration = float('inf')):
    #Note: GLEW doesn't work outside of the visualization thread. 
    #here a threadCall is used to simulate the sensors.
    world,sim,sensor = set_up_world()
    robot = world.robot(0)
    vis.add("world",world)
    vis.add("sensor",sensor)

    vis.show()
    time.sleep(0.5)

    def simulate_in_vis_thread():
        t0 = time.time()
        #sensor.kinematicSimulate(world,0.01)
        sim.simulate(0.01)
        sim.updateWorld()
        t1 = time.time()
        # print("multithreaded sensor simulation took time",t1-t0)
        run_sensing_tests(sensor)

    while vis.shown():
        vis.lock()
        robot.randomizeConfig()
        sim.controller(0).setPIDCommand(robot.getConfig(),[0.0]*7)
        sim.updateWorld()  #restore the world state
        vis.unlock()
        vis.threadCall(simulate_in_vis_thread)
        time.sleep(0.01)
        if vis.animationTime() > duration:
            break

    vis.show(False)
    vis.kill()


def test_fallback(duration = float('inf')):
    #Note: GLEW doesn't work outside of the visualization thread. 
    #The below code falls back to the non-accelerated sensor simulation.
    world,sim,sensor = set_up_world()
    robot = world.robot(0)
    vis.add("world",world)
    vis.add("sensor",sensor)

    vis.show()
    time.sleep(0.5)

    while vis.shown():
        vis.lock()
        robot.randomizeConfig()
        t0 = time.time()
        sensor.kinematicSimulate(world,0.01)
        t1 = time.time()
        print("Fallback sensor simulation took time",t1-t0)
        #sim.controller(0).setPIDCommand(robot.getConfig(),[0.0]*7)
        vis.unlock()

        pc = sensing.camera_to_points(sensor,points_format='Geometry3D',all_points=True,color_format='rgb')
        T = sensor.getTransformWorld()
        pc.setCurrentTransform(*T)
        vis.add("pc",pc)
        time.sleep(0.01)
        if vis.animationTime() > duration:
            break

    vis.show(False)
    vis.kill()


print("------------------------------------------------------------------------------")
print("cameratest.py: This script tests the sensing module with a simulated camera.")
print("Multiple methods may be tested, including physical simulation, kinematic")
print("simulation, multithreaded, and slow fallback simulation.")
print("------------------------------------------------------------------------------")

#test_sim_simulate()
#test_kinematic_simulate()
test_multithreaded()
#test_fallback()