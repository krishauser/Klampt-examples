#!/usr/bin/python3
#This file also contains functionalities to control the gripper. This builds upon python-urx.

import logging
import multiprocessing as mp
import signal
import socket
import time
from copy import copy, deepcopy

import numpy as np
from klampt.math import vectorops
from scipy import signal as scipysignal

import rtde
import ur5_constants
from utils import in_limits, SharedMap
from ur5_dashboard import UR5DashboardClient

logger = logging.getLogger(__name__)  # pylint: disable=invalid-name


# NOTE: SETPOINT_NONE is special. It will never be passed to UR, indicates failure
SETPOINT_NONE     = 0
SETPOINT_HALT     = 1
SETPOINT_POSITION = 2
SETPOINT_VELOCITY = 3
SETPOINT_WRENCH   = 4
SET_FREE_DRIVE    = 5

# Integer registers
REG_SETPOINT = 0
REG_TYPE = 1
REG_FREE_DRIVE_ACTIVE = 3
REG_COMPLIANCE = 4
REG_ZERO_FTSENSOR = 10

# Double input registers
REG_TARGET = 0
REG_ACCELERATION = 6
REG_LOOKAHEAD = 7
REG_DAMPING = 7
REG_GAIN = 8
REG_G = 9
REG_LIMITS = 12
REG_FORCE_TARGET = 12
REG_TASK_FRAME = 18

# Double output registers
REG_JOINT_TORQUE = 0    # 0-5


class UR5RTDEDriver:
    def __init__(self, host : str, IO_buffer : SharedMap, filter_flag : bool, qmin, qmax, vmin, vmax, dashboard_client : UR5DashboardClient, **kwargs):
        """
        An interface to the UR5 using the UR Real Time Data Exchange (RTDE) protocol.

        Parameters:
            host:               The UR5 controller IP address
            IO_buffer:          Shared memory data structure for going between here and limbController.
            filter_flag:        Whether the wrench should be filtered.
            qmin:               Software joint position limit (min).
            qmax:               Software joint position limit (max).
            vmin:               Software joint velocity limit (min).
            vmax:               Software joint velocity limit (max).
            dashboard_client:   Dashboard client for communicating with the UR (for resetting protective
                                stops right now, nothing else). Instance of Motion.ur5dashboard.UR5DashboardClient.
        Keyword arguments:
            rtde_port:          port for RTDE, default 30004
            command_port:       port for commands, default 30002
        
        Use IO_buffer to communicate with the RTDE program.
        """
        self.dashboard_client = dashboard_client

        self.qmin = qmin
        self.qmax = qmax
        self.vmin = vmin
        self.vmax = vmax

        self._robot_host = host
        self._rtde_port = kwargs.pop('rtde_port', 30004)
        self._command_port = kwargs.pop('command_port', 30002)
        self._gripper = kwargs.pop('gripper', False)
        self._speed_scale = None

        self._cog = kwargs.pop('cog', [0.0,0.0,0.0])
        self._payload = kwargs.pop('payload', 0.0)
        self._gravity = kwargs.pop('gravity', [0, 0, 9.82])

        self._version = None

        self.IO_buffer = IO_buffer

        self._start_time = None
        self.last_t = 0

        #stuff needed for threading
        self._conn = None
        self._max_speed_scale = None
        self._sock = None

        self.IO_buffer['running'] = 0

        # Configuration that is guaranteed to be in the defined joint limits.
        self._safe_config = None

        self.c = np.array([10, 10, 10, 7.5, 7.5, 7.5])

        self._filter_flag = filter_flag
        if self._filter_flag:
            self._filtered_wrench = []
            self.histories = [list() for i in range(6)]
            self._history_length = 25

            ## filter parameters
            Wn=0.1
            [self.b2,self.a2]=scipysignal.butter(3,Wn,'lowpass')
            [self.b,self.a]=scipysignal.butter(3,(0.03, 0.06),'bandstop')
        self.accum_current = np.zeros(6)
        
    def start(self):
        """
        Start ur5 controller in a subprocess.
        """
        control_process = mp.Process(target = self._start, args = [])
        control_process.start()

    def _start(self):
        # initialize RTDE
        """
        General purpose registers (input):
        Integer: (0-23)
            0:   setpoint_id
            1:   target_type (control mode)
            2:   [UNUSED]
            3:   free_drive_active (freedrive mode)
            4-9: compliance (for force control)
            10-23: [UNUSED] (??? maybe some gripper stuff idrk)

        Double: (0-23)
            0-5:   target (target configuration, velocity, etc)
            6:     acceleration (of base joint, for speedj)
            7 (1): lookahead (for Model Predictive Control, servoj)
            7 (2): damping (for force control)
            8:     gain (proportional gain for MPC)
            9-11:  gravity vector
            # 12-17: limits (for force control) (CURRENTLY DO NOT USE. HARDCODED)
            12-17: target_velocity (for force control)
            18-23: task_frame (for force control)

        For info on special input registers see:
            https://www.universal-robots.com/articles/ur/interface-communication/real-time-data-exchange-rtde-guide/
        """
        try:
            self._conn = rtde.RTDE(self._robot_host, self._rtde_port)
            self._conn.connect()
            self._version = self._conn.get_controller_version()
            
            # configure outputs (URControl -> Python)
            self._conn.send_output_setup(['timestamp', 'target_q', 'actual_q', 'target_qd',
                    'actual_qd', 'target_qdd', 'target_moment', 'target_speed_fraction',
                    'actual_TCP_force', 'actual_current', 'target_current',
                    'safety_status_bits','robot_status_bits','safety_mode', 'robot_mode']
                    + ['output_double_register_{}'.format(i) for i in range(6)],
                    frequency=250)

            # configure inputs (Python -> URControl)
            input_names = (['input_int_register_{}'.format(i) for i in range(24)]
                            + ['input_double_register_{}'.format(i) for i in range(24)]
                            + ['speed_slider_mask', 'speed_slider_fraction'])
            self.registers = self._conn.send_input_setup(input_names, 
                                                        ['INT32']*24 + ['DOUBLE']*24 + ['UINT32', 'DOUBLE'])
            for name in input_names:
                self.registers.__dict__[name] = 0

            # start RTDE
            self._conn.send_start()

            # start the controller program
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.connect((self._robot_host, self._command_port))
            self._program = CONTROLLER_PROGRAM.format(cog = self._cog,
                                         payload = self._payload,
                                         gravity = self._gravity,
                                         gripper_flag = self._gripper,
                                         setpoint_none = SETPOINT_NONE,
                                         setpoint_halt = SETPOINT_HALT,
                                         setpoint_position = SETPOINT_POSITION,
                                         setpoint_velocity = SETPOINT_VELOCITY,
                                         setpoint_wrench = SETPOINT_WRENCH,
                                         set_free_drive = SET_FREE_DRIVE,
                                         reg_setpoint = REG_SETPOINT,
                                         reg_type = REG_TYPE,
                                         reg_free_drive_active = REG_FREE_DRIVE_ACTIVE,
                                         reg_compliance = REG_COMPLIANCE,
                                         reg_zero_ftsensor = REG_ZERO_FTSENSOR,
                                         reg_target = REG_TARGET,
                                         reg_acceleration = REG_ACCELERATION,
                                         reg_lookahead = REG_LOOKAHEAD,
                                         reg_damping = REG_DAMPING,
                                         reg_gain = REG_GAIN,
                                         reg_g = REG_G,
                                         reg_limits = REG_LIMITS,
                                         reg_force_target = REG_FORCE_TARGET,
                                         reg_task_frame = REG_TASK_FRAME,
                                         reg_joint_torque = REG_JOINT_TORQUE)

            #logger.info('controller program:\n{}'.format(self._program))
            self._sock.sendall(self._program.encode('ascii') + b'\n')

            self._max_speed_scale = None
            self.state = None
            self.controlLoop()

        finally:
            pass

    def resend_program(self):
        self._sock.sendall(self._program.encode('ascii') + b'\n')
        
    def controlLoop(self):
        # NOTE: IGNORE SIGINT!
        signal.signal(signal.SIGINT, lambda signal, frame: None)
        setpoint_number = 0
        with self.IO_buffer.lock():
            self.IO_buffer['running'] = 1
        while True:
            t1 = time.time()
            state = self._conn.receive()
            terr = time.time() - t1
            if terr > 0.05:
                print("TIMEOUT EXCEEDED {}".format(terr))
                print(self.get_input_register(REG_TYPE, 'int'))
            if self.state is None and state is None:
                time.sleep(0.01)
                continue
            with self.IO_buffer.lock():
                stop = self.IO_buffer.get('stop_flag')
            if stop <= 0:
                self._sock.sendall(b'stop program\n')
                self._sock.close()
                self._sock = None
                self._conn.send_pause()
                self._conn.disconnect()
                self._conn = None
                print("disconnecting")
                break
            else:
                with self.IO_buffer.lock():
                    self.IO_buffer['stop_flag'] = stop - 1
            if state is not None:
                # TODO: estimate state in the case of no sensor feedback
                self.state = state

            # honor speed scaling set when program started
            if self._max_speed_scale is None:
                self._max_speed_scale = self.state.target_speed_fraction
                self._speed_scale = self.state.target_speed_fraction

            # kick watchdog
            setpoint_number += 1
            self.set_register(REG_SETPOINT, setpoint_number, 'int')
# invoke update
            self._update(self.state)
        #if this loop exits, disconnect


    def setHalt(self):
        """
        Stop motion of the UR5.
        """
        self.set_register(REG_TYPE, SETPOINT_HALT, 'int')
        
    def setFreedriveMode(self, freedrive_mode):
        """
        Enables or disables freedrive on the UR5.

        Parameters:
            freedrive_mode: Truthy value to enable freedrive, else dissables.
        """
        if freedrive_mode:
            freedrive_mode = 1
        else:
            freedrive_mode = 0

        self.set_register(REG_TYPE, SET_FREE_DRIVE, 'int')
        self.set_register(REG_FREE_DRIVE_ACTIVE, freedrive_mode, 'int')

    def setPosition(self, q, lookahead=0.1, gain=400):
        """
        Set the UR5 to move in position mode to the specified position.
        Wraps an eventual `servoj` call.
        
        Parameters:
            q:          Target joint config.
            lookahead:  Parameter used for smoothing the path.
                        Higher = more smoothing (range 0.03 to 0.2, default 0.1)
            gain:       Proportional gain for following the target.
                        Higher = faster response (range 100 to 2000, default 300).

        Note: High gain with low lookahead may result in instability!
        """
        self.set_register(REG_TYPE, SETPOINT_POSITION, 'int')
        self.set_register(REG_GAIN, gain, 'double')
        self.set_register(REG_LOOKAHEAD, lookahead, 'double')
        self.l2r(q, REG_TARGET)

    def setVelocity(self, qd, qdd_base=10):
        """
        Set the UR5 to move in velocity mode to the specified velocity.
        Wraps an eventual `speedj` call.
        
        Parameters:
            qd:         Target joint velocities.
            qdd_base:   Acceleration of the base link, rad/s^2.
        """
        self.set_register(REG_TYPE, SETPOINT_VELOCITY, 'int')
        self.set_register(REG_ACCELERATION, qdd_base, 'double')
        self.l2r(qd, REG_TARGET)

    def setWrench(self, target, wrench, damping=0.5, task_frame=[0, 0, 0, 0, 0, 0],
                  compliance=[1, 1, 1, 1, 1, 1]):
        """
        Set the UR5 to perform a motion in force mode.
        Wraps a force_mode call.

        Parameters:
            target:     Target joint angles to drive to.
            wrench:     Target wrench in the task frame.
            damping:    How damped the robot motion will be I guess. Exactly how this works
                        was not specified but it is a number from 0 to 1.
                        Default: 0.5
            task_frame: Reference frame in which the wrench and compliance are expressed.
                        Relative to the robot base frame.
                        Order: [X, Y, Z, AX, AY, AZ] (translation, concat with axis-angle orientation.)
                        Default: [0, 0, 0, 0, 0, 0] (same as robot base frame).
            compliance: Degrees of freedom that will be compliant in the end effector.
                        An array of six values (0 or 1, 1 for compliant).
                        Order: [X, Y, Z, RX, RY, RZ].
                        Default: [1, 1, 1, 1, 1, 1] (All dofs compliant).
            limits:     For compliant DOF: Velocity limits (m/s, rad/s).
                        For non compliant DOF: Displacement limits (m, rad).
        """
        self.set_register(REG_TYPE, SETPOINT_WRENCH, 'int')
        self.l2r(wrench, REG_TARGET)
        self.l2r(task_frame, REG_TASK_FRAME)
        self.l2r(compliance, REG_COMPLIANCE, "int")
        #l2r(limits, self.limits_reg, REG_LIMITS)
        self.l2r(target, REG_FORCE_TARGET)
        self.set_register(REG_DAMPING, damping, 'double')

    #this function is not used
    #speed_scale set on teaching pendant directly
    def speed_scale(self, s=None):
        if s is not None:
            self._speed_scale = s

        return self._speed_scale

    def _update(self, state):
        buf = self.IO_buffer

        if self._start_time is None:
            self._start_time = state.timestamp
        t = state.timestamp - self._start_time
        dt = t - self.last_t
        self.last_t = t

        buf.lock_acquire()
        buf.copy_from_object(state)

        pstop = False
        if state.safety_status_bits & ur5_constants.PROTECTIVE_STOP_MASK:
            buf.lock_release()
            print("Arm has been protective stopped. Resetting in 6s...")
            time.sleep(6.0)
            for i in range(5):
                try:
                    self.dashboard_client.unlockProtectiveStop()
                    break
                except:
                    time.sleep(1)
                    print("Could not unlock... retry")
                    continue
            print("Resending URScript...")
            self.resend_program()

            state = self._conn.receive()
            self.state = state
            if state is None:
                return
            buf.lock_acquire()
            buf.copy_from_object(state)
            pstop = True
            # self.left_limb.controller.setFreeDrive(True)
            # print("Entering freedrive. You have 15 seconds until it is disabled.")
            # time.sleep(15.0)
            # self.left_limb.controller.setFreeDrive(False)
            # print("Exiting freedrive and protective stop procedure...")

        buf["connected"] = 1
        joint_torques = [self.get_output_register(state, REG_JOINT_TORQUE + i) for i in range(6)]
        buf["joint_torques"] = joint_torques
        
        if self._safe_config is None:
            self._safe_config = state.actual_q

        target_current = buf.get('target_current')
        target_torque = buf.get('target_moment')
        c = [a/b if b != 0 else 0 for a, b in zip(target_torque, target_current)]
        for i, cv in enumerate(c):
            if cv > 6 and cv < 12:
                self.c[i] = self.c[i]*0.9 + cv*0.1
        current_error = self.c * (np.array(target_current) - buf.get('actual_current'))
        self.accum_current = 0.96*self.accum_current + 0.04*current_error
        buf['current_error'] = self.accum_current

        #Add and filter wrench here
        if self._filter_flag:
            wrench = buf.get('actual_TCP_force')
            dat = wrench
            if len(self.histories[0]) < self._history_length:
                for i, x in enumerate(self.histories):
                    x.append(dat[i])
                buf['filtered_wrench'] = wrench
            else:
                #filtering all 6 of these takes about 0.1 ms
                _filtered_dat = [0.0]*6
                for i, x in enumerate(self.histories):
                    assert len(x) == self._history_length
                    x.pop(0)
                    x.append(dat[i])
                    _filtered_dat[i] = scipysignal.filtfilt(self.b, self.a, x)[self._history_length -1].tolist()
                buf['filtered_wrench'] =_filtered_dat[:6]

        if pstop:
            self.setPosition(self._safe_config, 0.2, 100)
            buf.lock_release()

            self._conn.send(self.registers)
            return

        if buf.get('zero_ftsensor'):
            self.set_register(REG_ZERO_FTSENSOR, 1, regtype="int")
            buf['zero_ftsensor'] = 0
        else:
            self.set_register(REG_ZERO_FTSENSOR, 0, regtype="int")

        if buf.get('use_soft_limit'):
            # TODO: velocity
            qmin = self.qmin
            qmax = self.qmax
        else:
            qmin = ur5_constants.MIN_JOINTS
            qmax = ur5_constants.MAX_JOINTS

        stop_robot = False
        control_mode = buf.get('control_mode')
        if control_mode == SETPOINT_NONE:
            #if no commands are set, go to the current position
            stop_robot = True
        elif control_mode == SETPOINT_HALT:
            self.setHalt()
        elif control_mode == SETPOINT_POSITION:
            q_commanded = buf.get('q_commanded')
            #print("RTDE q_command:", q_commanded)
            if self.isFormatted(q_commanded):
                if not in_limits(q_commanded, qmin, qmax):
                    print("RTDE out of bounds,", qmin, qmax)
                    buf['control_mode'] = SETPOINT_NONE
                    stop_robot = True
                else:
                    lookahead = buf.get("lookahead")
                    delta = np.array(q_commanded) - state.actual_q
                    qd_limit = min(self.vmax) * lookahead
                    max_qd = np.max(np.abs(delta))
                    if max_qd > qd_limit:
                        print("RTDE velocity limit")
                        delta = delta * qd_limit / max_qd
                    self.setPosition(state.actual_q + delta, lookahead)
            else:
                print("RTDE: malformed q_com")
                buf['control_mode'] = SETPOINT_NONE
                stop_robot = True
                print(q_commanded)
                print("Warning, improper position formatting. Halting")
        elif control_mode == SETPOINT_VELOCITY:
            qdot_commanded = buf.get('qdot_commanded')
            if self.isFormatted(qdot_commanded):
                if in_limits(qdot_commanded, self.vmin, self.vmax):
                    q_next = vectorops.madd(state.actual_q, qdot_commanded, dt)
                    #commanded velocity is rad/s
                    #only want to check next position limits of robot not gripper
                    #UR5_CL is the configuration length of just the UR5 robot = 6
                    if not in_limits(q_next, qmin, qmax):
                        buf['control_mode'] = SETPOINT_NONE
                        stop_robot = True
                        print("Warning, exceeding joint limits. Halting")
                    else:
                        self.setVelocity(qdot_commanded)
            else:
                buf['control_mode'] = SETPOINT_NONE
                stop_robot = True
                print("Warning, improper velocity formatting. Halting")
        elif control_mode == SETPOINT_WRENCH:
            q_commanded = buf.get('q_commanded')
            wrench_commanded = buf.get('wrench_commanded')
            if not in_limits(q_commanded, qmin, qmax):
                stop_robot = True
                print("Warning, exceeding joint limits. Halting")
            else:
                damping_commanded = buf.get('damping_commanded')
                task_frame = buf.get('task_frame')
                self.setWrench(q_commanded, wrench_commanded, damping_commanded, task_frame)
        elif control_mode == SET_FREE_DRIVE:
            self.setFreedriveMode(buf.get('free_drive_commanded'))

        #print(pinchness(state.actual_q))
        if not in_limits(state.actual_q, qmin, qmax):
            print("RTDE: HARD STOP - JOINT LIMITS", state.actual_q, qmin, qmax)
            stop_robot = True
        #if in_pinch(state.actual_q, pinch_radius = 0.12):
        #    stop_robot = True
        if not stop_robot:
            self._safe_config = state.actual_q

        # NOTE: slightly jank. but whatever man
        if stop_robot:
            self.setPosition(self._safe_config, 0.2, 100)

        self.setGravity(buf.get('gravity'))

        # clamp speed scale
        self._speed_scale = max(min(self._speed_scale, self._max_speed_scale), 0)

        self.registers.speed_slider_mask = 1
        self.registers.speed_slider_fraction = self._speed_scale

        # send gravity
        self.l2r(self._gravity, REG_G, "double", 3)

        buf.lock_release()

        self._conn.send(self.registers)

    def isFormatted(self, val):
        #do formatting
        if val:
            if len(val) == ur5_constants.NUM_JOINTS:
                return True
        else:
            print("Error, val: ", val, " is not formatted correctly")
        return False

    def setGravity(self,g):
        self._gravity = deepcopy(g)

    @property
    def version(self):
        return self._version


    ########################################
    # Some utility functions
    ########################################

    def r2l(self, base=0, regtype="double", n=6):
        """
        Convert consecutive register values to a list.
        Parameters:
            base: Index to start from (inclusive)
        Return:
            List of 6 values read from the register.
        """
        ret = []
        for i in range(n):
            ret.append(self.registers.__dict__['input_{}_register_{}'.format(regtype, base + i)])
        return ret

    def l2r(self, input_list, base=0, regtype="double", n=6):
        """
        Write values in a list to consecutive registers.
        Parameters:
            input_list: List to read from.
            base:       Index to start from (inclusive)
        Return:
        """
        for i in range (n):
            self.registers.__dict__['input_{}_register_{}'.format(regtype, base + i)] = input_list[i]

    def set_register(self, regnum, value, regtype="double"):
        """
        Set a value in a register.
        Parameters:
            TODO
        """
        self.registers.__dict__['input_{}_register_{}'.format(regtype, regnum)] = value

    def get_input_register(self, regnum, regtype="double"):
        """
        Get a value in an input register.
        Parameters:
            TODO
        Return:
            The value.
        """
        return self.registers.__dict__['input_{}_register_{}'.format(regtype, regnum)]

    def get_output_register(self, registers, regnum, regtype="double"):
        """
        Get a value in an output register.
        Parameters:
            TODO
        Return:
            The value.
        """
        return registers.__dict__['output_{}_register_{}'.format(regtype, regnum)]


#RTDE script sent to UR5
CONTROLLER_PROGRAM = '''
stop program

socket_send_string("close popup", "internal")
socket_send_byte(10, "internal")

def rtde_control_loop():
    # Tare the FT sensor
    zero_ftsensor()
    # constants
    SETPOINT_TIMEOUT  = 125
    SETPOINT_NONE     = {setpoint_none}
    SETPOINT_HALT     = {setpoint_halt}
    SETPOINT_POSITION = {setpoint_position}
    SETPOINT_VELOCITY = {setpoint_velocity}
    SET_FREE_DRIVE    = {set_free_drive}
    SETPOINT_WRENCH   = {setpoint_wrench}
    CONTROL_PERIOD = 0.004
    RTDE_WATCHDOG_FREQUENCY = 1

    # robotiq gripper
    GRIPPER_FLAG = {gripper_flag}

    # integer registers
    REG_SETPOINT = {reg_setpoint}
    REG_TYPE = {reg_type}
    REG_FREE_DRIVE_ACTIVE = {reg_free_drive_active}
    REG_COMPLIANCE = {reg_compliance}
    REG_ZERO_FTSENSOR = {reg_zero_ftsensor}

    # input double registers
    REG_TARGET = {reg_target}
    REG_ACCELERATION = {reg_acceleration}
    REG_LOOKAHEAD = {reg_lookahead}
    REG_DAMPING = {reg_damping}
    REG_GAIN = {reg_gain}
    REG_G = {reg_g}
    REG_LIMITS = {reg_limits}
    REG_FORCE_TARGET = {reg_force_target}
    REG_TASK_FRAME = {reg_task_frame}
    
    # output double registers
    REG_JOINT_TORQUE = {reg_joint_torque}

    # I/O configuration
    set_standard_analog_input_domain(0, 1)
    set_standard_analog_input_domain(1, 1)
    set_tool_analog_input_domain(0, 1)
    set_tool_analog_input_domain(1, 1)
    set_analog_outputdomain(0, 0)
    set_analog_outputdomain(1, 0)
    set_input_actions_to_default()

    if GRIPPER_FLAG:
       set_tool_voltage(24)
       set_tool_communication(True,115200,0,1,1.5,3.5)
    else:
       set_tool_voltage(0)
    end

    # tool configuration
    set_payload_cog([{cog[0]}, {cog[1]}, {cog[2]}])
    set_payload({payload})
    set_gravity([{gravity[0]}, {gravity[1]}, {gravity[2]}])

    setpoint_number = read_input_integer_register(REG_SETPOINT)
    last_setpoint_number = setpoint_number
    missed_setpoints = 0

    # WHAT IS THIS FOR?
    rtde_set_watchdog("input_int_register_0", RTDE_WATCHDOG_FREQUENCY, "stop")

    in_force_mode = 0
    tick_num = 0

    while True:
        setpoint_number = read_input_integer_register(REG_SETPOINT)
        if setpoint_number == last_setpoint_number:
            missed_setpoints = missed_setpoints + 1
        else:
            missed_setpoints = 0
        end
        tick_num = tick_num + 1
        last_setpoint_number = setpoint_number

        if missed_setpoints >= SETPOINT_TIMEOUT:
            popup("setpoint timeout", title="PyUniversalRobot", error=True)
            halt
        end

        should_zero = read_input_integer_register(REG_ZERO_FTSENSOR)
        if should_zero == 1:
            zero_ftsensor()
        end

        # update the setpoint
        write_output_integer_register(0, setpoint_number)

        joint_torques = get_joint_torques()
        write_output_float_register(REG_JOINT_TORQUE + 0, joint_torques[0])
        write_output_float_register(REG_JOINT_TORQUE + 1, joint_torques[1])
        write_output_float_register(REG_JOINT_TORQUE + 2, joint_torques[2])
        write_output_float_register(REG_JOINT_TORQUE + 3, joint_torques[3])
        write_output_float_register(REG_JOINT_TORQUE + 4, joint_torques[4])
        write_output_float_register(REG_JOINT_TORQUE + 5, joint_torques[5])

        target = [0, 0, 0, 0, 0, 0]
        target[0] = read_input_float_register(REG_TARGET + 0)
        target[1] = read_input_float_register(REG_TARGET + 1)
        target[2] = read_input_float_register(REG_TARGET + 2)
        target[3] = read_input_float_register(REG_TARGET + 3)
        target[4] = read_input_float_register(REG_TARGET + 4)
        target[5] = read_input_float_register(REG_TARGET + 5)

        G = [0,0,0]
        G[0] = read_input_float_register(REG_G + 0)
        G[1] = read_input_float_register(REG_G + 1)
        G[2] = read_input_float_register(REG_G + 2)
        set_gravity(G)

        type = read_input_integer_register(REG_TYPE)
        if type == SETPOINT_WRENCH:
            if tick_num % 5 == 1:

                force_mode_set_gain_scaling(2.0)

                damping = read_input_float_register(REG_DAMPING)
                force_mode_set_damping(damping)
                # TODO: Write a helper?
                compliance = [0, 0, 0, 0, 0, 0]
                compliance[0] = read_input_integer_register(REG_COMPLIANCE + 0)
                compliance[1] = read_input_integer_register(REG_COMPLIANCE + 1)
                compliance[2] = read_input_integer_register(REG_COMPLIANCE + 2)
                compliance[3] = read_input_integer_register(REG_COMPLIANCE + 3)
                compliance[4] = read_input_integer_register(REG_COMPLIANCE + 4)
                compliance[5] = read_input_integer_register(REG_COMPLIANCE + 5)
                task_frame = p[0, 0, 0, 0, 0, 0]
                task_frame[0] = read_input_float_register(REG_TASK_FRAME + 0)
                task_frame[1] = read_input_float_register(REG_TASK_FRAME + 1)
                task_frame[2] = read_input_float_register(REG_TASK_FRAME + 2)
                task_frame[3] = read_input_float_register(REG_TASK_FRAME + 3)
                task_frame[4] = read_input_float_register(REG_TASK_FRAME + 4)
                task_frame[5] = read_input_float_register(REG_TASK_FRAME + 5)

                # hardcoded for now...
                limits     = [5, 5, 5, 5, 5, 5]
                # limits[0] = read_input_float_register(REG_LIMITS + 0)
                # limits[1] = read_input_float_register(REG_LIMITS + 1)
                # limits[2] = read_input_float_register(REG_LIMITS + 2)
                # limits[3] = read_input_float_register(REG_LIMITS + 3)
                # limits[4] = read_input_float_register(REG_LIMITS + 4)
                # limits[5] = read_input_float_register(REG_LIMITS + 5)

                force_mode(task_frame, compliance, target, 2, limits)
                in_force_mode = 1
            end

            if in_force_mode == 1:
                target_pos = [0, 0, 0, 0, 0, 0]
                target_pos[0] = read_input_float_register(REG_FORCE_TARGET + 0)
                target_pos[1] = read_input_float_register(REG_FORCE_TARGET + 1)
                target_pos[2] = read_input_float_register(REG_FORCE_TARGET + 2)
                target_pos[3] = read_input_float_register(REG_FORCE_TARGET + 3)
                target_pos[4] = read_input_float_register(REG_FORCE_TARGET + 4)
                target_pos[5] = read_input_float_register(REG_FORCE_TARGET + 5)

                # hardcoded for now...
                lookahead = 0.02
                gain = 300
                acceleration = 10
                #speedj(target_pos, acceleration, CONTROL_PERIOD)
                servoj(target_pos, 0, 0, CONTROL_PERIOD, lookahead, gain)
            end
        elif in_force_mode == 1:
            in_force_mode = 0
            end_force_mode()
        end

        if type == SETPOINT_WRENCH:
            do_nothing = 0
        elif type == SET_FREE_DRIVE:
            free_drive_active = read_input_integer_register(REG_FREE_DRIVE_ACTIVE)
            if free_drive_active == 1:
                freedrive_mode()
            else:
                end_freedrive_mode()
            end
        elif type == SETPOINT_HALT:
            # issue command
            popup("halt command issued", title="PyUniversalRobot", error=True)
            halt
        elif type == SETPOINT_POSITION:
            # read lookahead and gain parameters
            lookahead = read_input_float_register(REG_LOOKAHEAD)
            if lookahead > 0.2:
                # In case we transitioned partially from force mode to position mode
                lookahead = 0.2
            end
            gain = read_input_float_register(REG_GAIN)

            # issue command
            # NOTE: acceleration and velocity arguments are ignored
            servoj(target, 0, 0, CONTROL_PERIOD, lookahead, gain)
        elif type == SETPOINT_VELOCITY:
            # read acceleration parameter
            acceleration = read_input_float_register(REG_ACCELERATION)

            # issue command
            speedj(target, acceleration, CONTROL_PERIOD)
        else:
            # alert and quit
            popup("unknown setpoint type received", title="PyUniversalRobot", error=True)
            halt
        end
    end
end
'''
