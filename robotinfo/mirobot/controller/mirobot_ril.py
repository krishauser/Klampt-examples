from klampt.control import RobotInterfaceBase, RobotInterfaceCompleter, OmniRobotInterface
from klampt.math import vectorops,so3
from mirobot import Mirobot
import time
import math

class MirobotInterface (RobotInterfaceBase):
    """Connects with the Wlkata Mirobot.  Default construction will auto-scan
    ports.
    
    Need to install the patched Wlkata Python API from source::

        git clone https://github.com/krishauser/mirobot-py.git
        cd mirobot-py
        python3 setup.py install
    
    Args:
        portname: passed to Mirobot device
        baudrate: passed to Mirobot device
        debug: passed to Mirobot device
        valve_pwm_values: passed to Mirobot device
        pump_pwm_values: passed to Mirobot device

    """
    def __init__(self,*mirobot_args,**mirobot_kwargs):
        RobotInterfaceBase.__init__(self)
        self._mirobot = Mirobot(*mirobot_args,**mirobot_kwargs,autoconnect=False)
        self._start_time = 0

    def initialize(self):
        if not self._mirobot.is_connected:
            print("Connecting to Mirobot...")
            self._mirobot.connect()
        if not self._mirobot.is_connected:
            print("Mirobot not connected")
            return False
        self._start_time = time.time()
        self._mirobot.update_status()
        if self._mirobot.status.state == 'Alarm':
            self._mirobot.home_simultaneous(wait=True)
            self._mirobot.update_status()
        else:
            self._mirobot.go_to_zero(wait=True)
        return True
        
    def close(self):
        if self._mirobot.is_connected:
            self._mirobot.disconnect()
    
    def numJoints(self):
        return 6
    
    def controlRate(self):
        return 10
    
    def status(self, joint_idx=None):
        if self._mirobot.status.state in ['Idle','Run']:
            return 'ok'
        return self._mirobot.status.state
    
    def isMoving(self):
        return self._mirobot.status.state == 'Run'
    
    def beginStep(self):
        self._mirobot.update_status()
        print(self._mirobot.status.state)
    
    def endStep(self):
        pass
    
    def functionCall(self,func,*args,**kwargs):
        if func == 'home':
            self._mirobot.home_simultaneous()
        else:
            raise ValueError("Invalid function call")
    
    def sensedPosition(self):
        angles = self._mirobot.angle
        deg = [angles.a1,angles.a2,angles.a3,angles.a4,angles.a5,angles.a6]
        return [math.radians(d) for d in deg]
    
    def commandedPosition(self):
        return self.sensedPosition()
    
    def setPosition(self,q):
        if len(q) != 6:
            raise ValueError("Invalid length of position")
        deg = [round(math.degrees(v),3) for v in q]
        self._mirobot.go_to_axis(*deg,wait=False)
    
    def moveToPosition(self,q,speed=1.0):
        if len(q) != 6:
            raise ValueError("Invalid length of position")
        deg = [round(math.degrees(v),3) for v in q]
        self._mirobot.go_to_axis(*deg,speed=speed*self._mirobot.default_speed,wait=False)
    
    def getToolCoordinates(self):
        return [0,0,0]

    def sensedCartesianPosition(self, frame='world'):
        return self.commandedCartesianPosition(frame)

    def commandedCartesianPosition(self, frame='world'):
        if frame not in ['world','base']:
            raise ValueError("Can't do cartesian position outside of world/base")
        cartesian = self._mirobot.cartesian
        rpy = [math.radians(cartesian.a), math.radians(cartesian.b), math.radians(cartesian.c)]
        t = [cartesian.x*0.001,cartesian.y*0.001,cartesian.z*0.001]
        R = so3.from_rpy(rpy)
        return (R,t)
    
    def setCartesianPosition(self, xparams, frame='world'):
        if frame not in ['world','base']:
            raise ValueError("Can't do cartesian position outside of world/base")
        R,t = xparams
        tmm = [round(v*1000,3) for v in t]
        rpy_rad = so3.rpy(R)
        rpy = [round(math.degrees(v),3) for v in rpy_rad]
        rpy = [v if v >= 0 else v+360 for v in rpy]
        self._mirobot.go_to_cartesian_ptp(tmm[0],tmm[1],tmm[2],rpy[0],rpy[1],rpy[2],wait=False)
    
    def moveToCartesianPosition(self, xparams, speed=1.0, frame='world'):
        if frame not in ['world','base']:
            raise ValueError("Can't do cartesian position outside of world/base")
        R,t = xparams
        tmm = [round(v*1000,3) for v in t]
        rpy_rad = so3.rpy(R)
        rpy = [round(math.degrees(v),3) for v in rpy_rad]
        rpy = [v if v >= 0 else v+360 for v in rpy]
        self._mirobot.go_to_cartesian_ptp(tmm[0],tmm[1],tmm[2],rpy[0],rpy[1],rpy[2],speed=speed*self._mirobot.default_speed,wait=False)
    
    #The go_to_cartesian_lin function is terrible
    # def moveToCartesianPositionLinear(self, xparams, speed=1.0, frame='world'):
    #     if frame not in ['world','base']:
    #         raise ValueError("Can't do cartesian position outside of world/base")
    #     R,t = xparams
    #     tmm = [round(v*1000,3) for v in t]
    #     rpy_rad = so3.rpy(R)
    #     rpy = [round(math.degrees(v),3) for v in rpy_rad]
    #     rpy = [v if v >= 0 else v+360 for v in rpy]
    #     self._mirobot.go_to_cartesian_lin(tmm[0],tmm[1],tmm[2],rpy[0],rpy[1],rpy[2],speed=speed*self._mirobot.default_speed,wait=False)

class MirobotGripperInterface(RobotInterfaceBase):
    def __init__(self,mirobot):
        self._mirobot = mirobot
        self._pwm_range = [int(v) for v in self._mirobot.valve_pwm_values]
    def numJoints(self):
        return 1
    def controlRate(self):
        return 10
    def setPosition(self,q):
        assert len(q)==1
        pwm_val = int(self._pwm_range[0] + q[0]*(self._pwm_range[1]-self._pwm_range[0]))
        self._mirobot.set_valve(pwm_val, wait=False)
    def sensedPosition(self):
        pwm_val = self._mirobot.status.valve_pwm
        to_range = (pwm_val - self._pwm_range[0])/(self._pwm_range[1]-self._pwm_range[0])
        return [to_range]
    def commandedPosition(self):
        return self.sensedPosition()

class MirobotPumpInterface(RobotInterfaceBase):
    def __init__(self,mirobot):
        self._mirobot = mirobot
        self._valve_range = [int(v) for v in self._mirobot.valve_pwm_values]
        self._pump_range = [int(v) for v in self._mirobot.pump_pwm_values]
    def numJoints(self):
        return 1
    def controlRate(self):
        return 10
    def setPosition(self,q):
        assert len(q)==1
        if q[0] > 0.5: #turn on
            self._mirobot.set_air_pump(self._pump_range[0], wait=False)
            self._mirobot.set_valve(self._valve_range[1], wait=False)
        else:
            self._mirobot.set_air_pump(self._pump_range[1], wait=False)
            self._mirobot.set_valve(self._valve_range[0], wait=False)
    def sensedPosition(self):
        pwm_val = self._mirobot.status.pump_pwm
        to_range = (pwm_val - self._pump_range[0])/(self._pump_range[1]-self._pump_range[0])
        return [to_range]
    def commandedPosition(self):
        return self.sensedPosition()


def make(robotModel,portname=None,debug=True):
    end_effector = None
    if robotModel.numDrivers()==7:
        end_effector = 'gripper'
    elif robotModel.numLinks()==8:
        end_effector = 'pump'
    print("Detected Mirobot end effector:",end_effector)
    #raw interface
    # res = MirobotInterface(portname=portname,debug=debug)
    # res._klamptModel = robotModel
    # return res
    #completed interface
    if end_effector is None:
        res = RobotInterfaceCompleter(MirobotInterface(portname=portname,debug=debug))
        res._klamptModel = robotModel
        res._base._klamptModel = robotModel
    else:
        res = OmniRobotInterface(robotModel)
        arm_interface = MirobotInterface(portname=portname,debug=debug)
        res.addPhysicalPart("arm",arm_interface)
        if end_effector == 'gripper':
            ee_interface = MirobotGripperInterface(arm_interface._mirobot)
            res.addPhysicalPart("gripper",ee_interface)
        else:
            ee_interface = MirobotPumpInterface(arm_interface._mirobot)
            res.addPhysicalPart("pump",ee_interface)
    return res
