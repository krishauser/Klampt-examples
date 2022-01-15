from klampt.math import vectorops,so3,se3
from klampt.control.robotinterface import RobotInterfaceBase
import ur5_driver
from utils import clamp_limits, in_limits, SharedMap
from ur5_dashboard import UR5DashboardClient
import ur5_constants
import time, math
import numpy as np
from copy import copy, deepcopy
import warnings


class UR5RobotInterface(RobotInterfaceBase):
    def __init__(self, host, **kwargs):
        """
        - host: the UR5 controller IP address

        UR5 keyword arguments
        - rtde_port: port for RTDE, default 30004
        - command_port: port for commands, default 30002
        """
        self.IO_buffer = SharedMap({
                'control_mode': (int,0),
                'use_soft_limit': (int,0),    # Hard limit (UR) or soft limit (avoid singularity)
                'actual_q': (float,6),     # q_curr
                'actual_qd': (float,6),    # qdot_curr
                'q_commanded': (float,6),
                'lookahead': (float,0),      # Position control lookahead.
                'qdot_commanded': (float,6),
                'gravity': (float,3),
                'wrench': (float,6),
                'filtered_wrench': (float,6),
                'wrench_commanded': (float,6),
                'damping_commanded': (float,0),
                'compliance': (int,6),
                'limits': (float,6),
                'task_frame': (float,6),
                'free_drive_commanded': (int,0),
                'running': (int,0),
                'connected': (int,0),
                'stop_flag': (int,0),
                'zero_ftsensor': (int,0),

                'target_q': (float,6),
                'target_qd': (float,6),
                'target_qdd': (float,6),
                'target_moment': (float,6),
                'actual_current': (float,6),
                'target_current': (float,6),
                'current_error': (float,6),
                'joint_torques': (float,6),
                'actual_TCP_force': (float,6),
                'safety_status_bits': (int,0),
                'robot_status_bits': (int,0),
                'safety_mode': (int,0),
                'robot_mode': (int,0),
                'timestamp': (float,0)
            },lock=False)
        self.IO_buffer['stop_flag'] = 1000
        #self.IO_buffer.set('control_mode', SETPOINT_HALT)

        # Connect to Dashboard
        print("Attempting to connect to UR5 dashboard")
        self.dashboard_client = UR5DashboardClient(host)
        self.dashboard_client.connect()

        self._cog = kwargs.get('cog', (0.0,0.0,0.0))
        self._payload = kwargs.get('payload', 0.0)

        self._filter_flag = False
        self.qmin, self.qmax = copy.copy(ur5_constants.MIN_JOINTS),copy.copy(ur5_constants.MAX_JOINTS)
        self.vmin, self.vmax = copy.copy(ur5_constants.MIN_VEL),copy.copy(ur5_constants.MAX_VEL)
        # NOTE: Danger of sharing object between processes -- dashboard client must be stateless!
        self.ur5 = ur5_driver.UR5RTDEDriver(host, self.IO_buffer, self._filter_flag, self.qmin, self.qmax, self.vmin, self.vmax, self.dashboard_client, **kwargs)

        self._q_curr = None
        self._qdot_curr = [0, 0, 0, 0, 0, 0]
        self._gravity = kwargs.pop('gravity', [0, 0, 9.82])
        self.IO_buffer['gravity'] = self._gravity
        self._started = False

        #Filter wrench
        self._wrench = [0.0]*6
        self._filtered_wrench = [0.0]*6
        self._current_error = [0.0]*6

        self._joint_torques = [0.0]*6
        self._joint_currents = [0.0]*6
        self._joint_current_targets = [0.0]*6

        self._target_qdd = [0.0]*6

        self._speed_fraction=0.0

        self.ur5_safety_status_names =['normal','reduced','protective_stopped','recovery','safeguard_stop','system_emergency_stop','robot_emergency_stop','emergency_stop','violation','fault','xx','xx','xx','xx']	
        self.ur5_robot_mode_names = ['no_controller','disconnected','confirm_safety','booting','power_off','power_on','idle','backdrive','running','updating_firmware']	
        self.safety_status_bits = 0	
        self.robot_status_bits = 0	
        self.robot_mode = -1	
        self.safety_mode = 0
        ##Pause/resume
        self._paused = False
        self._update_time = None
        self._start_time = None
        self.properties['asynchronous'] = True

    def setGravityCompensation(self,gravity,load,load_com):
        if self._started:
            if load != self._payload:
                warnings.warn("gravity compensation can only change gravity vector after initialize() is called")
            if load_com != self._cog:
                warnings.warn("gravity compensation can only change gravity vector after initialize() is called")    
        self._gravity = gravity
        self._payload = load
        self._cog = load_com
        with self.IO_buffer.lock():
            self.IO_buffer['gravity'] = gravity

    def initialize(self):
        self.activateArm()  
        self.ur5.start()
        success = False
        for i in range(200):
            with self.IO_buffer.lock():
                if self.IO_buffer['connected']:
                    success = True
                    break                
            time.sleep(0.1)
            if i % 10 == 1:
                print("Waiting for connection from UR5...")
        if not success:
            print("Unable to read from UR5")
            return False
        self.beginStep()
        self._start_time = self._update_time
        #TODO: why is this here
        with self.IO_buffer.lock():
            self.IO_buffer['stop_flag'] = 10000

        current_config=self.commandedPosition()
        print("Current Config: " + str(current_config))
        self.setPosition(current_config)
        self.endStep()
        self._started = True
        return True

    def close(self):
        with self.IO_buffer.lock():
            self.IO_buffer['stop_flag'] = 0

    def beginStep(self):
        self._update_time = time.time()
        #update current notion of state
        buf = self.IO_buffer
        with buf.lock():
            buf['stop_flag'] = 100
            self._q_curr = buf['actual_q']
            self._qdot_curr = buf['actual_qd']
            self._wrench = buf['actual_TCP_force']
            self._filtered_wrench = buf['filtered_wrench']
            self._joint_torques = buf['joint_torques']
            self._joint_torque_targets = buf['target_moment']
            self._joint_currents = buf['actual_current']
            self._joint_current_targets = buf['target_current']
            self.control_timestamp = buf['timestamp']
            self.safety_status_bits = buf['safety_status_bits']
            self.robot_status_bits = buf['robot_status_bits']
            self.robot_mode = buf['robot_mode']
            self.safety_mode = buf['safety_mode']
            self._target_qdd = buf['target_qdd']
            
    def numJoints(self):
        return ur5_constants.NUM_JOINTS
    
    def controlRate(self):
        return ur5_constants.CONTROL_RATE
    
    def clock(self):
        return time.time()-self._start_time
    
    def estop(self):
        print("TODO: activate actual estop")
        self.softStop()

    def softStop(self):
        self.setPosition(self.commandedPosition())
        self._paused = True

    def reset(self):
        status = self.status()
        if status == 'paused':
            self._paused = False
            return True
        elif status == 'protective_stopped':
            self.dashboard_client.unlockProtectiveStop()
            return True
        elif status in ['violation','fault','safeguard_stop']:
            self.dashboard_client.restartSafety()
            return self.activateArm()
        elif status == 'ok':
            return True
        #TODO: wait for a while?
        return False
    
    def sensors(self):
        return ['wrench','filtered_wrench']
    
    def enabledSensors(self):
        return self.sensors()
    
    def sensorMeasurements(self,sensor):
        if sensor == 'wrench':
            return self._wrench
        if sensor == 'filtered_wrench':
            return self._filtered_wrench
        raise ValueError("No sensor named {}".format(sensor))
    
    def sensorUpdateTime(self,sensor):
        return self._update_time-self._start_time

    def status(self):
        with self.IO_buffer.lock():
            running = self.IO_buffer['running']
            connected = self.IO_buffer['connected']
        if not connected:
            return 'disconnected'
        if not running:
            return 'no_controller'
        if self.robot_mode >= 0:
            return self.ur5_robot_mode_names[self.robot_mode + 1]
        if self.safety_mode > 1:
            return self.ur5_safety_status_names[self.safety_mode - 1]
        if self.safety_status_bits & ur5_constants.PROTECTIVE_STOP_MASK:
            return "protective_stopped"
        if self._paused:
            return 'paused'
        return 'ok'

    def isMoving(self):
        return vectorops.norm(self._qdot_curr) > 0.0001

    def sensedPosition(self):
        return self._q_curr

    def sensedVelocity(self):
        return self._qdot_curr
    
    def sensedTorque(self):
        return self._joint_torques
    
    def sensedCurrent(self):
        return self._joint_currents
    
    def commandedPosition(self):
        with self.IO_buffer.lock():
            res = copy.copy(self.IO_buffer['q_commanded'])
        return res

    def get_current_error(self, filtered=False):
        if filtered and self._filter_flag:
            return self.current_error
        target_torque = self._joint_torque_targets
        target_current = self._joint_current_targets
        c = [a/b if b != 0 else 0 for a, b in zip(target_torque, target_current)]
        return vectorops.mul(c, vectorops.sub(target_current, self._joint_currents))

    def getTargetJointAccelerations(self):
        return self._target_qdd
    
    def functionCall(self,func,*args):
        """
        Possible values for func:
        * 'zero_ft_sensor': Set the wrench offset to the current measured wrench.
        """
        if func == 'zero_ft_sensor':
            with self.IO_buffer.lock():
                self.IO_buffer['zero_ftsensor'] = 1
        else:
            raise ValueError("Invalid function call {}".format(func))

    def setSetting(self, item, value):
        if item == 'lookahead':
            lookahead = value
            buf = self.IO_buffer
            with buf.lock():
                buf['lookahead'] = lookahead
        elif item == 'speed_fraction':
            self._speed_fraction = value
        else:
            raise NotImplementedError("Invalid setting {}".format(item))
    
    def getSetting(self, item):
        if item == 'lookahead':
            buf = self.IO_buffer
            with buf.lock():
                return buf['lookahead']
        elif item == 'speed_fraction':
            return self._speed_fraction
        else:
            raise NotImplementedError("Invalid setting {}".format(item))
    
    def setControlMode(self, mode, *args, **kwargs):
        if mode == 'free_drive':
            self.setFreeDrive(True)
        elif mode == 'pause':
            self._paused = True
            self.setConfig(self._q_curr)
        else:
            self._paused = False
            self.setFreeDrive(False)

    def setPosition(self, q_in):
        """
        Set the position of this arm in joint space.

        Parameters:
            q_in: Position of each joint in radians.
        """
        if not self._paused:
            buf = self.IO_buffer
            with buf.lock():
                buf['control_mode'] = ur5_driver.SETPOINT_POSITION
                buf['q_commanded'] = q_in 

    def setVelocity(self, dq_in):
        """
        Set the velocity of this arm in joint space.

        Parameters:
            dq_in: Velocity of each joint in rad/s.
        """
        if not self._paused:
            buf = self.IO_buffer
            with buf.lock():
                buf['control_mode'] = ur5_driver.SETPOINT_VELOCITY
                buf['qdot_commanded'] = dq_in

    def setWrench(self, target, wrench_in, damping=0.5, task_frame=((1, 0, 0, 0, 1, 0, 0, 0, 1), (0, 0, 0))):
        """
        Set the arm to wrench mode with the given parameters.
        NOTE: The input wrench must be expressed in the robot's base frame!
        TODO: expose more parameters!

        Parameters:
            target:     Target joint angles to drive to.
            wrench_in:  Target wrench that the arm will try to reach.
            damping:    Damping to use for arm motion (0 to 1).
            task_frame: Frame in which to apply the wrench, relative to the robot base.
                        Pass in a klampt se3 transform. (R, t) where R is in column-major order.
        """
        if not self._paused:
            if len(wrench_in) != 6: raise ValueError("Invalid wrench")
            buf = self.IO_buffer
            with buf.lock():
                buf['control_mode']=ur5_driver.SETPOINT_WRENCH
                buf['q_commanded']=target
                buf['wrench_commanded']=wrench_in
                buf['damping_commanded']=damping
                buf['task_frame']=[*(task_frame[1]), *(so3.rotation_vector(task_frame[0]))]
                    
    def setFreeDrive(self, active):
        """
        Set the limb freedrive mode.
        """
        if not self._paused:
            buf = self.IO_buffer
            with buf.lock():
                buf['control_mode']=ur5_driver.SET_FREE_DRIVE
                buf['free_drive_commanded']=1 if active else 0

    def activateArm(self):
        """
        Activates (powers on and releases brakes) the limb
        NOTE: For this to work, both control boxes must be turned on and any e-stop should be released. One arm will be turned on at a time.
        """
        if self.dashboard_client.robotmode() == "Robotmode: RUNNING":
            print("Arm already activated")
            return True
        if self.dashboard_client.safetyStatus() == "Safetystatus: PROTECTIVE_STOP":
            self.dashboard_client.unlockProtectiveStop()
        if self.dashboard_client.safetyStatus() != "Safetystatus: NORMAL":
            print("WARNING - LIMBCONTROLLER: Restarting safety -- {} != NORMAL".format(self.dashboard_client.safetyStatus()))
            self.dashboard_client.restartSafety()  # This takes ~3s, absorb into next wait
            
        N_RETRIES = 20
        for i in range(1+N_RETRIES):
            if self.dashboard_client.safetyStatus() == "Safetystatus: NORMAL":
                break
            # Causes of a non-normal safety status:
            # Both arms aren't on, E-stop is active, etc. This should wait until both arms are activated
            if i == N_RETRIES:
                print("WARNING - LIMBCONTROLLER: Not activating arm because safety status {} != NORMAL. FAILED".format(self.dashboard_client.safetyStatus()))
                return False
            print("WARNING - LIMBCONTROLLER: Not activating arm because safety status {} != NORMAL. Retrying in 1 second".format(self.dashboard_client.safetyStatus()))
            time.sleep(1.0)

        print("Starting arm activation sequence")
        if self.dashboard_client.robotmode() == "Robotmode: POWER_OFF": 
            self.dashboard_client.powerOn()
        self.dashboard_client.powerOn()
        for i in range(1+N_RETRIES):
            if self.dashboard_client.robotmode() == "Robotmode: IDLE":
                break
            if i == N_RETRIES:
                print("WARNING - LIMBCONTROLLER: Not releasing brake because {} != IDLE".format(self.dashboard_client.robotmode()))
                return False
            time.sleep(1.0)
            print("Robot powered on - {}".format(self.dashboard_client.robotmode()))
        self.dashboard_client.brakeRelease()
        for i in range(1+N_RETRIES):
            if self.dashboard_client.robotmode() == "Robotmode: RUNNING":
                break
            if i == N_RETRIES:
                print("WARNING - LIMBCONTROLLER: failed arm activation {} != RUNNING".format(self.dashboard_client.robotmode()))
                return False
            time.sleep(1.0)
            print("Robot brake releasing - {}".format(self.dashboard_client.robotmode()))
        print("LIMBCONTROLLER: Finished arm activation sequence")
        return True
        
    def deactivateArm(self):
        """
        Deactivates the limb
        """
        self.dashboard_client.powerOff()
        self.dashboard_client.shutdown()
     
    
if __name__ == "__main__":
    # Testing Dashboard Client
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--addr', action='store', type=str, required=True)
    ap.add_argument('--gripper', action='store', type=str, required=False)
    args = ap.parse_args()

    ur5 = UR5RobotInterface(args.addr, gripper=False, gravity=[4.91,-4.91,-6.93672],type = args.gripper,payload =0.72,cog = [0,0,0.05])    # from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
    
