#!/usr/bin/python
import sys
import klampt
from klampt import vis
from klampt.model.robotinfo import RobotInfo
from klampt.vis.glrobotprogram import GLSimulationPlugin


class MyGLViewer(GLSimulationPlugin):
    def __init__(self,world):
        GLSimulationPlugin.__init__(self,world)
        rinfo = RobotInfo.load("../../robotinfo/robotiq_3finger_sim.json")
        rinfo.configureSimulator(self.sim,0)
        assert len(self.sim.actuatorEmulators) == 1,"Assumed to have one actuator emulator"
        print(self.sim.actuatorEmulators)
        self.robotiqEmulator = self.sim.actuatorEmulators[0][0]
        assert self.robotiqEmulator.__class__.__name__ == 'Emulator',"first emulator isn't the RobotiqEmulator?"

    def control_loop(self):
        #Put your control handler here
        
        #right now, just sets g to an oscillation between 0 and 199
        #TODO: build a RobotControllerBlock that outputs qcmd to the emulator
        g = int(self.sim.getTime()*50.0)
        maxval = 120
        if int(g/maxval)%2 == 1:
            g = maxval-1 - g%maxval
        else:
            g = g % maxval
        #print g
        g = [g,g,g]

        self.robotiqEmulator.send_command(g,scissor=30)


if __name__ == "__main__":
    print("""robotiqtest.py: A program to test the behavior of the RobotiQ
    emulator.  Right now it just opens and closes the gripper repeatedly.

    Press s to toggle simulation.""")
    world = klampt.WorldModel()

    if not world.readFile('robotiq.xml'):
        print("robotiq.xml couldn't be read, exiting")
        exit(1)

    viewer = MyGLViewer(world)
    vis.setWindowTitle("Robotiq gripper test")
    vis.run(viewer)
