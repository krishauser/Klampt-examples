import math

from klampt.control.robotinterface import RobotInterfaceBase
from kortex_api.autogen.messages import Base_pb2


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
