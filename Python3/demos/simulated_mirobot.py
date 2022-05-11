from klampt.sim.simulation import SimpleSimulator
from klampt.model.robotinfo import RobotInfo
from klampt.control import RobotInterfaceBase,RobotInterfaceCompleter,StepContext
from klampt.control.interop import RobotInterfacetoVis
from klampt.control.simrobotinterface import SimPositionControlInterface
from klampt.control.blocks.robotcontroller import RobotControllerBlock
from klampt import *
from klampt import vis
import matplotlib.pyplot as plt
import numpy as np

WORLD_FILE = '../../data/mirobot_6blocks.xml'
ROBOTINFO_FILE = '../../robotinfo/mirobot/mirobot_vacuum_sim.json'

class VacuumBindingBlock(RobotControllerBlock):
    """RobotControllerBlock interface to Mirobot vacuum"""
    def __init__(self):
        self.value = 0
        self.flow = 0
    def advance(self,**outputs):
        if 'flow' in outputs:
            self.flow = outputs['flow']
            del outputs['flow']
        outputs['vacuum'] = self.value
        return outputs
    def setValue(self,value):
        self.value = value
    def setOn(self):
        self.value = 1
    def setOff(self):
        self.value = 0


class VacuumControlInterface(RobotInterfaceBase):
    """RIL interface to Mirobot vacuum"""
    def __init__(self,block):
        RobotInterfaceBase.__init__(self)
        self.block = block
    def controlRate(self):
        return 100.0
    def numJoints(self, part = None):
        return 1
    def initialize(self):
        return True
    def setPosition(self,q):
        self.block.setValue(q[0])
    def sensedPosition(self):
        return [self.block.value]
    def sensors(self):
        return ['flow']
    def sensorMeasurements(self, name):
        if name != 'flow':
            raise ValueError("Invalid sensor")
        return [self.block.flow]


class UnionControlInterface(RobotInterfaceBase):
    """A simple multi-part control interface that doesn't support
    simultaneous control between parts.
    """
    def __init__(self):
        RobotInterfaceBase.__init__(self)
        self._partInterfaces = dict()
        self._partIndices = dict()
        self._numJoints = 0
    def addPart(self,name,partInterface,inds=None):
        if name in self._partIndices:
            raise ValueError("Can't add two parts with the same name: {}".format(name))
        self._partInterfaces[name] = partInterface
        if inds is not None:
            self._partIndices[name] = inds
        else:
            self._partIndices[name] = list(range(self.numJoints,self.numJoints+partInterface.numJoints()))
        self._numJoints += partInterface.numJoints()
    def numJoints(self):
        return self._numJoints
    def parts(self):
        return self._partIndices
    def partInterface(self,name):
        return self._partInterfaces[name]
    def initialize(self):
        for (n,iface) in self._partInterfaces.items():
            if not iface.initialize():
                print("Interface",n,"failed to initialize")
                return False
        return True
    def close(self):
        for (n,iface) in self._partInterfaces.items():
            iface.close()
    def controlRate(self):
        return max(iface.controlRate() for iface in self._partInterfaces.values())
    def beginStep(self):
        for iface in self._partInterfaces.values():
            iface.beginStep()
    def endStep(self):
        for iface in self._partInterfaces.values():
            iface.endStep()


class SimulatedMirobotWithVacuumAndSensor:
    """The main API for interacting with the simulated Mirobot with vacuum
    gripper and simulated flow sensor.
    """
    def __init__(self,controller,world,sim,robotinfo):
        self.controller = controller
        self.world = world
        self.sim = sim
        self.robotinfo = robotinfo
        self.arm = controller.partInterface("arm")
        self.vacuum = controller.partInterface("vacuum")
        cam_model = world.robot(0).sensor('rgbd_camera')
        self.image_w,self.image_h = int(cam_model.getSetting('xres')),int(cam_model.getSetting('yres'))
        self.has_rgb = int(cam_model.getSetting('rgb'))
        self.has_depth = int(cam_model.getSetting('depth'))
        self.next_camera_time = 0
        self.last_camera_measurements = None
        self.numIters = 0
    def robotModel(self) -> RobotModel:
        return self.world.robot(0)
    def beginStep(self) -> None:
        if self.numIters == 0:
            #this needs to be here so that the OpenGL context can be initailized before the first sensor call is extracted
            print("SimulatedMirobotWithVacuumAndSensor: Initialize controller")
            if not self.controller.initialize():
                raise RuntimeError("Couldn't initialize controller")
            print("SimulatedMirobotWithVacuumAndSensor: Sensors:",self.arm.sensors())
            vacuum_local_pos = [0,0,-0.0300]
            self.arm.setToolCoordinates(vacuum_local_pos)
    
        self.controller.beginStep()
    def endStep(self) -> None:
        self.controller.endStep()
        self.numIters += 1
    def close(self) -> None:
        self.controller.close()
    def toggleVacuum(self) -> None:
        """Turns the vacuum on or off."""
        q = self.vacuum.commandedPosition()
        if q[0] > 0.5:
            self.setVacuumOff()
        else:
            self.setVacuumOn()
    def setVacuumOn(self):
        self.vacuum.setPosition([1])
    def setVacuumOff(self):
        self.vacuum.setPosition([0])
    def getVacuumCommand(self):
        """Returns the activation of the vacuum (0 off, 1 on)."""
        return self.vacuum.commandedPosition()[0]
    def getVacuumFlow(self):
        """Returns the value from the flow sensor (0 minimum, 1 maximum)."""
        return self.vacuum.sensorMeasurements('flow')[0]
    def setArmPosition(self,q):
        """Sets the arm configuration.  Can either be a Klampt model configuration
        or a minimal (6) set of driver values."""
        if len(q) == self.world.robot(0).numLinks():
            q = self.arm.configFromKlampt(q)
        self.arm.moveToPosition(q)
    def sensedArmConfig(self):
        """Returns the arm configuration (only the actuated joints)."""
        return self.arm.sensedPosition()
    def commandedArmConfig(self):
        """Returns the commanded configuration (only the actuated joints)."""
        return self.arm.commandedPosition()
    def hasNewRgbdImages(self):
        """Returns true if the RGBD images will update."""
        return (self.sim.getTime() >= self.next_camera_time)
    def rgbdImages(self):
        """Extracts the RGB-D data from the sensor.  Returns a color,depth pair."""
        if self.sim.getTime() < self.next_camera_time:
            return self.last_camera_measurements

        measurements = self.arm.sensorMeasurements('rgbd_camera')
        w,h = self.image_w,self.image_h
        rgb = None
        depth = None
        if self.has_rgb and len(measurements) > 0:
            argb = np.asarray(measurements[0:w*h]).reshape(h,w).astype(np.uint32)
            rgb = np.zeros((h,w,3),dtype=np.uint8)
            rgb[:,:,0] = np.right_shift(np.bitwise_and(argb,0x0ff0000), 16)
            rgb[:,:,1] = np.right_shift(np.bitwise_and(argb,0x00ff00), 8)
            rgb[:,:,2] =                np.bitwise_and(argb,0x00000ff)
        
        if self.has_depth and len(measurements) > 0:
            start = (w*h if self.has_rgb else 0)
            depth = np.asarray(measurements[start:start+w*h]).reshape(h,w)
        
        camera_dt = 1.0/30.0
        self.last_camera_measurements = (rgb,depth)
        if self.next_camera_time + camera_dt < self.sim.getTime():
            self.next_camera_time = self.sim.getTime() + camera_dt
        else:
            self.next_camera_time = self.next_camera_time + camera_dt
        return (rgb,depth)


def createRobotController() -> SimulatedMirobotWithVacuumAndSensor:
    """Creates the robot controller configured for simulated emulation of the mirobot."""
    world = WorldModel()
    if not world.loadFile(WORLD_FILE):
        raise IOError("Couldn't read {}".format(WORLD_FILE))
    sim = SimpleSimulator(world)
    rinfo = RobotInfo.load(ROBOTINFO_FILE)
    rinfo.configureSimulator(sim,0)
    vacuumBlock = VacuumBindingBlock()
    sim.setController(0,vacuumBlock)
    motorController = SimPositionControlInterface(sim.controller(0),sim,rinfo)
    vacuumController = VacuumControlInterface(vacuumBlock)
    controller = UnionControlInterface()
    controller.addPart("arm",RobotInterfaceCompleter(motorController),[0,1,2,3,4,5])
    controller.addPart("vacuum",RobotInterfaceCompleter(vacuumController),[6])
    return SimulatedMirobotWithVacuumAndSensor(controller,world,sim,rinfo)


if __name__ == '__main__':
    controller = createRobotController()
    
    controllerVis = RobotInterfacetoVis(controller.arm)
    doShowCamera = False
    plotShown = False
    im = None

    def showCamera():
        global doShowCamera
        doShowCamera = not doShowCamera

    def initVis():
        for i in range(controller.world.robot(0).numLinks()):
            controller.world.robot(0).link(i).appearance().setSilhouette(0)
        for i in range(controller.world.numRigidObjects()):
            controller.world.rigidObject(i).appearance().setSilhouette(0)
        for i in range(controller.world.numTerrains()):
            controller.world.terrain(i).appearance().setSilhouette(0)
        vis.add("world",controller.world)
        vis.add("target",controller.robotModel().getConfig(),color=(1,1,0,0.5))
        vis.edit("target")
        vis.addAction(controller.toggleVacuum,'Toggle vacuum','v')
        vis.addAction(showCamera,'Toggle camera','c')
        
    def loopVis():
        global plotShown,im
        with StepContext(controller):
            #reads the visualization configuration
            qdes = vis.getItemConfig("target")
            
            #drives the robot there
            controller.setArmPosition(qdes)

            #print the flow sensor if the vacuum is on
            if controller.getVacuumCommand() > 0:
                print("Flow:",controller.getVacuumFlow())

            if doShowCamera and controller.hasNewRgbdImages():
                #update the Matplotlib window if the sensor is working
                rgb,depth = controller.rgbdImages()
                if rgb is not None:
                    #funky stuff to make sure that the image window updates quickly
                    if not plotShown:
                        im = plt.imshow(rgb)
                        plt.show(block=False)
                        plotShown = True
                    else:
                        im.set_array(rgb)
                        plt.gcf().canvas.draw()
                                        
            controllerVis.update()
            
    def closeVis():
        controller.close()

    #maximum compability with Mac
    vis.loop(initVis,loopVis,closeVis)
    