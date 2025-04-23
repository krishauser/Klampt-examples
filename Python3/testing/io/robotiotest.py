import klampt
from klampt import vis
from klampt.math import so3,se3,vectorops
from klampt.model import sensing
import os


world = klampt.WorldModel()
world.readFile("../../../data/baxter_apc.xml")
robot = world.robot(0)
sensor = robot.addSensor("camera","CameraSensor")
sensor.setSetting("rgb","1")
sensor.setSetting("depth","1")
sensor.setSetting("xres","640")
sensor.setSetting("yres","480")
sensor.setSetting("zmin","0.1")
sensor.setSetting("zmax","100")
robot.saveFile("baxter_with_camera_temp.rob")
