from klampt.control.robotinterface import RobotInterfaceBase
from utilities import DeviceConnection
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
from kortex_api.autogen.client_stubs.DeviceConfigClientRpc import DeviceConfigClient
from kortex_api.autogen.client_stubs.GripperCyclicClientRpc import GripperCyclicClient
from kortex_api.autogen.client_stubs.InterconnectCyclicClientRpc import InterconnectCyclicClient
from kortex_api.RouterClient import RouterClientSendOptions
from kortex_api.autogen.messages import Session_pb2, Base_pb2, Common_pb2, InterconnectCyclic_pb2
import weakref
import math
import os
import time

class KinovaGen3RobotInterface(RobotInterfaceBase):
    """An Klamp't interface layer for a Kinova Gen3 robot. 

    Pass the constructed interface to RobotInterfaceCompleter().
    """
    def __init__(self,ip='192.168.1.10',username='admin',password='admin'):
        RobotInterfaceBase.__init__(self,name='Kinova Gen3 7DOF',
                                    # HACK: path to model file is hard-code
                                    klamptModelFile=os.path.join(os.path.dirname(__file__),
                                                                 "../../../data/robots/kinova_with_robotiq_85.urdf"))
        self.ip = ip
        self.credentials = (username,password)
        self.router = None
        self.router_cyclic = None
        self.tstart = None
    
    def initialize(self):
        """Tries to connect to the robot.  Returns true if ready to send
        commands.  This should probably be the first method called.
        """
        print("KinovaGen3RobotInterface: Connecting to",self.ip,"...")
        self.connection = DeviceConnection(self.ip, port=DeviceConnection.TCP_PORT, credentials=self.credentials)
        
        try:
            self.router = self.connection.__enter__()
        except Exception:
            print("Couldn't start router...")
            self.connection = None
            self.router = None
            return False

        self.connection_cyclic = DeviceConnection(self.ip, port=DeviceConnection.UDP_PORT, credentials=self.credentials)
        try:
            self.router_cyclic = self.connection_cyclic.__enter__()
            pass
        except Exception:
            print("Couldn't start router_cyclic...")
            self.connection_cyclic = None
            self.router_cyclic = None
            self.connection.__exit__(None,None,None)
            self.router = None
            return False

        print("KinovaGen3RobotInterface: connection is set up!")
        self.base = BaseClient(self.router)
        self.base_cyclic = BaseCyclicClient(self.router_cyclic)
        self.interconnect_cyclic = InterconnectCyclicClient(self.router_cyclic)
        self.num_arm_joints = self.base.GetActuatorCount().count
        self.arm = _ArmInterface(weakref.proxy(self))
        self.gripper = _GripperInterface(weakref.proxy(self))
        self.gripper.initialize()

                # Create required services
        device_manager = DeviceManagerClient(self.router)
        device_handles = device_manager.ReadAllDevices()
        
        # Only actuators are relevant for this example
        actuator_count=0
        has_gripper = False
        has_interconnect = False
        for handle in device_handles.device_handle:
            if handle.device_type == Common_pb2.BIG_ACTUATOR or handle.device_type == Common_pb2.SMALL_ACTUATOR:
                actuator_count += 1
            elif handle.device_type == Common_pb2.GRIPPER:
                print("Got gripper",handle.device_identifier)
                has_gripper = True
            elif handle.device_type == Common_pb2.INTERCONNECT:
                print("Got interconnect",handle.device_identifier)
                has_interconnect = True
                self.interconnect = handle.device_identifier
            elif handle.device_type == Common_pb2.BASE:
                print("Got base")
            else:
                print("Got device",handle.device_type,handle.device_identifier)
                
        if actuator_count != 7:
            print("Uh... don't have 7 joints?")
            self.close()
            return False

        self.sendOption = RouterClientSendOptions()
        self.sendOption.andForget = False
        self.sendOption.delay_ms = 0
        self.sendOption.timeout_ms = 3
        self.feedback = None
        for iters in range(20):
            try:
                self.feedback = self.base_cyclic.RefreshFeedback(0,self.sendOption)
                break
            except Exception:
                time.sleep(0.05)

        self.tstart = time.time()
        self._status = 'ok'
        return True

    def close(self):
        if self.router is None:
            return True
        self.connection.__exit__(None,None,None)
        self.connection_cyclic.__exit__(None,None,None)
        self.connection=None
        self.connection_cyclic=None
        self.router=None
        self.router_cyclic=None
        return True

    def __del__(self):
        self.close()

    def beginStep(self):
        assert self.router
        t0 = time.time()
        try:
            self.feedback = self.base_cyclic.RefreshFeedback(0,self.sendOption)
        except Exception:
            pass
        self.gripper.beginStep()
        
    def endStep(self):
        pass

    def numJoints(self,part=None):
        """Returns the number of joints of the given part.  By default, this
        returns the number of actuated DOFs in the Klamp't model. 
        """
        if part is None:
            return self.num_arm_joints + 1
        elif part == 'arm':
            return self.num_arm_joints
        elif part == 'gripper':
            return 1
        return len(self.parts()[part])
    
    def controlRate(self):
        return 10 #Hz

    def parts(self):
        """Returns a dictionary of (part-name,configuration index list) pairs
        defining the named parts of the robot.

        Since this will be used a lot, make sure to declare your override with
        @functools.lru_cache.
        """
        return {None:list(range(self.numJoints())),'arm':list(range(self.num_arm_joints)),'gripper':list(range(self.num_arm_joints,self.num_arm_joints+1))}

    def partInterface(self,part=None,joint_idx=None):
        if part is None and joint_idx is None:
            return self
        if joint_idx is not None:
            raise NotImplementedError()
        if part == 'arm':
            return self.arm
        elif part == 'gripper':
            return self.gripper
        raise ValueError("Invalid part {}".format(part))

    def clock(self):
        return time.time() - self.tstart

    def estop(self):
        self.base.ApplyEmergencyStop()
        self._status = 'estop'

    def softStop(self):
        self.base.Stop()
        self._status = 'soft_stop'

    def reset(self):
        self.base.ClearFaults()
        self._status = 'ok'
        return True

    def status(self,part=None,joint_idx=None):
        if part is not None or joint_idx is not None:
            return self.getPartController(part,joint_idx).status()
        return self._status

    def sensedPosition(self):
        return self.arm.sensedPosition() + self.gripper.sensedPosition()

    def sensedVelocity(self):
        return self.arm.sensedVelocity() + self.gripper.sensedVelocity()

    def sensedTorque(self):
        return self.arm.sensedTorque() + self.gripper.sensedTorque()

    def setPosition(self,p,ttl=None):
        raise NotImplementedError("setPosition only works with arm part")

    def setVelocity(self,v,ttl=None):
        self.arm.setVelocity(v[:self.num_arm_joints],ttl)
        self.gripper.setVelocity(v[self.num_arm_joints:],ttl)

    def setTorque(self,t,ttl=None):
        raise NotImplementedError("setTorque only works with arm part")


class _ArmInterface(RobotInterfaceBase):
    def __init__(self,kinova_iface):
        self.kinova_iface = kinova_iface

    def sensedPosition(self):
        res = []
        for act in self.kinova_iface.feedback.actuators:
            v = math.radians(act.position)
            if v > math.pi:  #normalize to range [-pi,pi]
                v -= math.pi*2
            res.append(v)
        return res

    def sensedVelocity(self):
        return [math.radians(act.velocity) for act in self.kinova_iface.feedback.actuators]

    def sensedTorque(self):
        return [act.torque for act in self.kinova_iface.feedback.actuators]

    def setVelocity(self,v,ttl=None):
        assert len(v) == self.kinova_iface.num_arm_joints
        joint_speeds = Base_pb2.JointSpeeds()
        for i,x in enumerate(v):
            joint_speed = joint_speeds.joint_speeds.add()
            joint_speed.joint_identifier = i 
            joint_speed.value = math.degrees(x)
            if abs(joint_speed.value) > 90:
                raise ValueError("Can't send command more than 45 degrees per second?")
            #joint_speed.duration = 0 if ttl is None else ttl
            joint_speed.duration = 0   #not implemented yet in Kortex
            i = i + 1
        #TODO: use BaseCyclic
        self.kinova_iface.base.SendJointSpeedsCommand(joint_speeds)


class _GripperInterface(RobotInterfaceBase):
    def __init__(self,kinova_iface):
        self.kinova_iface = kinova_iface
        self.last_cmd = None

    def initialize(self):
        return True

    def sensedPosition(self):
        return [self.kinova_iface.feedback.interconnect.gripper_feedback.motor[0].position*0.01]
    
    def sensedVelocity(self):
        return [self.kinova_iface.feedback.interconnect.gripper_feedback.motor[0].velocity*0.01]
    
    def sensedTorque(self):
        return [self.kinova_iface.feedback.interconnect.gripper_feedback.motor[0].current_motor]

    def setPosition(self,v):
        #TODO: use BaseCyclic
        assert len(v)==1
        if self.last_cmd == (3,v):
            return
        cmd = Base_pb2.GripperCommand()
        cmd.mode = 3
        cmd.duration = 0
        finger = cmd.gripper.finger.add()
        finger.finger_identifier = 0
        finger.value = 1-v[0]
        self.kinova_iface.base.SendGripperCommand(cmd)

    def setVelocity(self,v,ttl=None):
        assert len(v)==1
        if self.last_cmd == (2,v):
            return
        self.last_cmd = (2,v[:])
        cmd = Base_pb2.GripperCommand()
        cmd.mode = 2
        #cmd.duration = 0 if ttl is None else ttl
        cmd.duration = 0  #not implemented yet
        finger = cmd.gripper.finger.add()
        finger.finger_identifier = 0
        finger.value = -v[0]*0.5
        self.kinova_iface.base.SendGripperCommand(cmd)
    

def make(_):
    #HACK: kinova robot interface requires no argument, add a dummy parameter
    """Interface used to refer to robot interfaces by file"""
    from klampt.control.robotinterfaceutils import RobotInterfaceCompleter
    return RobotInterfaceCompleter(KinovaGen3RobotInterface())