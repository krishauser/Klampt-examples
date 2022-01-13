import atexit
import math
import time
from threading import Thread, Lock, RLock
import threading
import numpy as np
from copy import copy
from klampt.control.robotinterface import RobotInterfaceBase
from klampt.control.utils import TimedLooper
import dynamixel_sdk

print(dynamixel_sdk.__file__)
DEGREE_2_RADIAN = math.pi / 180.0
TICK_TO_DEG = 360 / 4096.0
DEG_TO_TICK = 1.0 / TICK_TO_DEG
TICK_TO_RAD = TICK_TO_DEG * DEGREE_2_RADIAN
RAD_TO_TICK = 1.0 / TICK_TO_RAD
SPEED_RAD_S_PER_TICK = 0.011623892818282236   #converts tick speed to RAD/s
SPEED_TICK_PER_RAD_S = 1.0/SPEED_RAD_S_PER_TICK #converts RAD/s to tick speed
SAFE_TEMPERATURE = 60
SHUTDOWN_TEMPERATURE = 78
# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_MOVING_SPEED       = 32
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_PRESENT_SPEED   = 38
ADDR_MX_SHUTDOWN = 17
ADDR_MX_TORQUE_LIMIT = 34
ADDR_MX_PRESENT_TEMP = 43
ADDR_MX_TORQUE_LIMIT = 34
ADDR_MX_PRESENT_LOAD = 40
LEN_PRESENT_TEMP = 1
LEN_TORQUE_LIMIT = 2
LEN_PRESENT_LOAD = 2
# Protocol version
PROTOCOL_VERSION = 1.0               # See which protocol version is used in the Dynamixel

MONITOR_DT = 0.5

# Default setting
BAUDRATE = 57600             # Dynamixel default baudrate : 57600

TORQUE_ENABLE = 1                 # Value for enabling the torque
TORQUE_DISABLE = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 10           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold


class DynamixelInterface(RobotInterfaceBase):
    """
    """
    def __init__(self, deviceName, dynamixel_ids, zero_offsets = None, max_speeds = None):
        RobotInterfaceBase.__init__(self)
        self.portHandler = dynamixel_sdk.PortHandler(deviceName)
        self.dynamixel = dynamixel_sdk.PacketHandler(PROTOCOL_VERSION)
        self.group_read = dynamixel_sdk.GroupBulkRead(self.portHandler, self.dynamixel)
        self.dynamixel_ids = dynamixel_ids

        self.active = False    
        self.dt = 0.05 #20 Hz
        self.monitor_freq = int(MONITOR_DT / self.dt)
        self.step_count = 0
        self._status = ['ok']*len(dynamixel_ids)
        #for pause/resume
        self.paused = False
        #TODO: more error checking
        self.errors = ['voltage','angle limit','overheating','range','CheckSum','Overload','Instruction','NotUsed']
        if zero_offsets is None:
            zero_offsets = [0]*len(dynamixel_ids)
        if max_speeds is None:
            max_speeds = [math.pi*2]*len(dynamixel_ids)  #default 1 rev / s
        self.setSetting('zero_offsets',zero_offsets)
        self.setSetting('max_speeds',max_speeds)
        
        
    def initialize(self):
        assert not self.active
        # dynamixel init
        # Open ports
        try:
            if self.portHandler.openPort():
                print("DynamixelInterface: Succeeded to open the port")
            else:
                print("DynamixelInterface: Failed to open the port")
        except Exception as e:
            print("DynamixelInterface: Failed to open the port! Maybe the controller is not connected?")
            return False
           
        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("DynamixelInterface: Succeeded to change the baudrate")
        else:
            print("DynamixelInterface: Failed to change the baudrate")

        for id in self.dymamixel_ids:
            # Enable Torque 
            self.dynamixel.write1ByteTxRx(self.portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
            #SPEED
            self.dynamixel.write2ByteTxRx(self.portHandler, id, ADDR_MX_MOVING_SPEED, (int)(60))
            # init torque
            dxl_addparam_result = self.group_read.addParam(id, ADDR_MX_TORQUE_LIMIT, 10)
            print("DynamixelInterface: Torque limit",id,"add result",dxl_addparam_result)
  
        self.active = True
        return True
    
    def beginStep(self):
        if self.step_count % self.monitor_freq == 0:
            self._update_monitor()
        self.step_count += 1
    
    def status(self):
        for id,s in zip(self.dynamixel_ids,self._status):
            if s != 'ok':
                return '{} (motor {})'.format(s,id)
        return 'ok'
    
    def setSetting(self,key,value):
        if key == 'max_speeds':
            if len(value) != len(self.dynamixel_ids): raise ValueError("Invalid # of motors")
            self.max_speeds = [x * SPEED_TICK_PER_RAD_S for x in value]  #1 rev / s
            self.properties['velocity_limits'] = ([-v for v in self.max_speeds],self.max_speeds)
        elif key == 'zero_offsets':
            if len(value) != len(self.dynamixel_ids): raise ValueError("Invalid # of motors")
            self.zero_offsets = value
            self.properties['joint_limits'] = ([-v for v in self.zero_offsets],[math.radians(300)-v for v in self.zero_offsets])

    def sensedPosition(self):
        """
        Returns:
            A list of angles, in radians
        """
        res = []
        for id,zero in zip(self.dynamixel_ids,self.zero_offsets):
            dxl_present_position, dxl_comm_result, dxl_error = self.dynamixel.read2ByteTxRx(self.portHandler, id, ADDR_MX_PRESENT_POSITION)
            res.append(dxl_present_position * TICK_TO_RAD - zero)
        return res
    
    def sensedVelocity(self):
        """
        Returns:
            A list of joint velocities, in radians / s
        """
        res = []
        for id in self.dynamixel_ids:
            dxl_present_velocity, dxl_comm_result, dxl_error = self.dynamixel.read2ByteTxRx(self.portHandler, id, ADDR_MX_PRESENT_SPEED)
            res.append(dxl_present_velocity * SPEED_RAD_S_PER_TICK)
        return res
    
    def commandedPosition(self):
        return self.sensedPosition()
    
    def isMoving(self, joint_idx = None) -> bool:
        if joint_idx is not None:
            id = self.dynamixel_ids[joint_idx]
            dxl_present_position, dxl_comm_result, dxl_error = self.dynamixel.read2ByteTxRx(self.portHandler, id, ADDR_MX_PRESENT_POSITION)
            dxl_goal_position, dxl_comm_result, dxl_error = self.dynamixel.read2ByteTxRx(self.portHandler, id, ADDR_MX_GOAL_POSITION)
            if dxl_present_position != dxl_goal_position:
                return True
        else:
            for id in self.dynamixel_ids
                dxl_present_position, dxl_comm_result, dxl_error = self.dynamixel.read2ByteTxRx(self.portHandler, id, ADDR_MX_PRESENT_POSITION)
                dxl_goal_position, dxl_comm_result, dxl_error = self.dynamixel.read2ByteTxRx(self.portHandler, id, ADDR_MX_GOAL_POSITION)
                if dxl_present_position != dxl_goal_position:
                    return True
        return False

    def commandedVelocity(self):
        res = []
        for id in self.dynamixel_ids:
            dxl_commanded_speed, dxl_comm_result, dxl_error = self.dynamixel.read2ByteTxRx(self.portHandler, id, ADDR_MX_MOVING_SPEED)
            res.append(dxl_commanded_speed * SPEED_RAD_S_PER_TICK)
        qtgt = self.destinationPosition()
        qcur = self.commandedPosition()
        for i in range(len(res)):
            if qtgt[i] < qcur[i]:
                res[i] *= -1
            elif qtgt[i] == qcur[i]:
                res[i] = 0
        return res
    
    def destinationPosition(self):
        res = []
        for id,zero in zip(self.dynamixel_ids,self.zero_offsets):
            dxl_present_position, dxl_comm_result, dxl_error = self.dynamixel.read2ByteTxRx(self.portHandler, id, ADDR_MX_GOAL_POSITION)
            res.append(dxl_present_position * TICK_TO_RAD - zero)
        return res

    def setPosition(self, q):
        """
        Sets the motors to the specified location as soon as possible.
        """
        if self.paused: return
        # conversion degree/0.08789
        ticks = [int(((v+zero)/DEGREE_2_RADIAN)/0.08789) for (v,zero) in zip(q,self.zero_offsets)]
        for id,tick in zip(self.dynamixel_ids,ticks):
            tick = max(DXL_MINIMUM_POSITION_VALUE,min(tick,DXL_MAXIMUM_POSITION_VALUE))
            result,error = self.dynamixel.write2ByteTxRx(self.portHandler, id, ADDR_MX_GOAL_POSITION, tick)
    
    def setVelocity(self, dq, ttl=None):
        q = self.commandedPosition()
        if ttl is None:
            ttl = 1.0
        goalq = [x + ttl*v for (x,v) in zip(q,dq)]
        for i,x in enumerate(goalq):
            spd = int(SPEED_TICK_PER_RAD_S*abs(dq[i]))
            tgt = int((x+self.zero_offsets[i])*RAD_TO_TICK)
            spd = min(spd,1023)
            tgt = max(DXL_MINIMUM_POSITION_VALUE,min(tgt,DXL_MAXIMUM_POSITION_VALUE))
            result, error = self.dynamixel.write2ByteTxRx(self.portHandler, id, ADDR_MX_MOVING_SPEED, spd)
            result, error = self.dynamixel.write2ByteTxRx(self.portHandler, id, ADDR_MX_GOAL_POSITION, tgt)

    def moveToPosition(self, q, speed = 1.0):
        for i,x in enumerate(q):
            spd = int(SPEED_TICK_PER_RAD_S*speed*self.max_speed[i])
            tgt = int((x+self.zero_offsets[i])*RAD_TO_TICK)
            spd = min(spd,1023)
            tgt = max(DXL_MINIMUM_POSITION_VALUE,min(tgt,DXL_MAXIMUM_POSITION_VALUE))
            result, error = self.dynamixel.write2ByteTxRx(self.portHandler, id, ADDR_MX_MOVING_SPEED, spd)
            result, error = self.dynamixel.write2ByteTxRx(self.portHandler, id, ADDR_MX_GOAL_POSITION, tgt)

    def functionCall(self,func,*args):
        if func == 'pause':
            self.paused = True
            self.setPosition(self.commandedPosition())
        elif func == 'resume':
            self.paused = False
        elif func == 'disableMotors':
            if len(args) > 0:
                self._disableMotors(args[0])
            else:
                self._disableMotors()
    
    def softStop(self):
        self.paused = True
        self.setPosition(self.commandedPosition())
    
    def reset(self):
        if self.paused:
            self.paused = True
        if any(s != 'ok' for s in self._status): #keep waiting?
            return False
        self._enableMotors()
        return True
    
    def controlRate(self):
        return 1.0/self.dt

    def status(self):
        if self.paused: return 'paused'
        return 'ok'

    def close(self):
        return
    
    def _disableMotors(self, id=None):
        # Disable Dynamixel Torque
        if id is None:
            for id in self.dynamixel_ids:
                self.dynamixel.write1ByteTxRx(self.portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
        else:
            self.dynamixel.write1ByteTxRx(self.portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)    

    def _enableMotors(self, id=None):
        # Enable Dynamixel Torque
        if id is None:
            for id in self.dynamixel_ids:
                self.dynamixel.write1ByteTxRx(self.portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        else:
            self.dynamixel.write1ByteTxRx(self.portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)    

    def _update_monitor(self):
        try:
            dxl_comm_result = self.group_read.txRxPacket()
        except Exception as e:
            print('DynamixelInterface::: WARNING: Bulk TXRX failed because of {}'.format(e))
            self._status = ['communication_failed']*len(self.dynamixel_ids)
            return
        disp = False
        if dxl_comm_result != dynamixel_sdk.COMM_SUCCESS:
            print("Dynamixel monitor failure %s" % self.dynamixel.getTxRxResult(dxl_comm_result))
            self._status = ['communication_failed: %s' % self.dynamixel.getTxRxResult(dxl_comm_result)]*len(self.dynamixel_ids)
            return
        
        for i,id in enumerate(self.dymamixel_ids):
            if self.group_read.isAvailable(id, ADDR_MX_TORQUE_LIMIT, 2):
                torque_limit = self.group_read.getData(id, ADDR_MX_TORQUE_LIMIT, 2)
            else:
                disp = False
            if self.group_read.isAvailable(id, ADDR_MX_PRESENT_TEMP, 1):
                temp = self.group_read.getData(id, ADDR_MX_PRESENT_TEMP, 1)
            else:
                disp = False    
            if self.group_read.isAvailable(load, ADDR_MX_PRESENT_LOAD, 2):
                load = self.group_read.getData(load, ADDR_MX_PRESENT_LOAD, 2)
                if(load > 1024):
                    load = 1024 - load
                load = 100*(load/1024)
            else:
                disp = False
        
            if disp:
                print('{}: Torque Limit: {} | Temp: {} | Load: {}'.format(id,torque_limit,temp,load))
            
            if torque_limit == 0:
                if temp < SAFE_TEMPERATURE:  #safe now
                    with self._dynamixelLock:
                        self.dynamixel.write2ByteTxRx(self.portHandler, id, ADDR_MX_TORQUE_LIMIT, 700)
                        self.dynamixel.write1ByteTxRx(self.portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
                    self._status[i] = 'ok'
            
            if temp > SHUTDOWN_TEMPERATURE:
                with self._dynamixelLock:
                    self._disableMotors(id)
                    self.dynamixel.write2ByteTxRx(self.portHandler, id, ADDR_MX_TORQUE_LIMIT, 0)
                self._status[i] = 'overheat'

        print()

if __name__ == "__main__":
    import sys
    device = sys.argv[1]
    a = DynamixelInterface(device,[1,2])
    from klampt.control.robotinterfaceutils import StepContext
    if not a.initialize():
        print("Unable to initialize dynamixel")
    
    def shut():
        a.close()

    import atexit
    atexit.register(shut)

    while True:
        with StepContext(a):
            print(a.sensedPosition())
            a.setPosition([math.radians(160),math.radians(270)])
        a.wait()
        with StepContext(a):
            a.setPosition([math.radians(180),math.radians(290)])
        a.wait()

    # a.shutdown()
