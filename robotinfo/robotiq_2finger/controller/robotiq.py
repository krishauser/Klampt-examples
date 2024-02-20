"""
Driver for communicating with the parallel Robotiq grippers
via the UR5 interface over port 63352
"""

import socket
import threading
import time
from enum import Enum
from collections import OrderedDict
from klampt.control.robotinterface import RobotInterfaceBase
from klampt.control.robotinterfaceutils import RobotInterfaceCompleter,ThreadedRobotInterface

class RobotiqParallelInterface(RobotInterfaceBase):
    """
    Communicates with a Robotiq 2-finger gripper via socket.

    Use setPosition, moveToPosition, setVelocity, or setPID to send commands.
    
    Implementation uses string commands to set / get variables. Blocking.
    Suggest wrapping this in a ThreadedRobotInterface.
    """
    
    def __init__(self, hostname, port = 63352, socket_timeout = 0.1):
        """Constructor. 

        :param hostname: Hostname or ip.
        :param port: Port.
        :param socket_timeout: Timeout for blocking socket operations.
        """
        super().__init__()

        self._hostname = hostname
        self._port = port
        self._socket_timeout = socket_timeout

        self.socket = None
        self.command_lock = threading.Lock()
        self.properties['joint_limits'] = ([0],[1])
        self.properties['velocity_limits'] = ([0],[1])  #TODO: calibrate these to actual velocities
        self.properties['torque_limits'] = ([0],[1])
        self._min_position = 0
        self._max_position = 255
        self._min_speed = 0
        self._max_speed = 255
        self._min_force = 0
        self._max_force = 255
        self._commandedTorque = 255

    def initialize(self):
        assert self.socket is None,"initialize() was called multiple times?"
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self._hostname, self._port))
        self.socket.settimeout(self._socket_timeout)

        self.reset()
        return True

    def close(self):
        self.socket.close()
        self.socket = None
    
    def controlRate(self):
        #TODO: what's a good rate?
        return 10

    def status(self):
        active = self._get_var(self.ACT)
        status = self._get_var(self.STA)
        if active == 0:
            return 'inactive'
        else:
            fault = self._get_var(self.FLT)
            if fault:
                return 'fault'
            if status == 3:
                return 'ok'
            elif status == 0:
                return 'resetting'
            else:
                return 'activating'

    def reset(self):
        """Resets the activation flag in the gripper, and sets it back to one, clearing previous fault flags.
        :param auto_calibrate: Whether to calibrate the minimum and maximum positions based on actual motion.
        The following code is executed in the corresponding script function
        def rq_activate(gripper_socket="1"):
            if (not rq_is_gripper_activated(gripper_socket)):
                rq_reset(gripper_socket)

                while(not rq_get_var("ACT", 1, gripper_socket) == 0 or not rq_get_var("STA", 1, gripper_socket) == 0):
                    rq_reset(gripper_socket)
                    sync()
                end

                rq_set_var("ACT",1, gripper_socket)
            end
        end
        def rq_activate_and_wait(gripper_socket="1"):
            if (not rq_is_gripper_activated(gripper_socket)):
                rq_activate(gripper_socket)
                sleep(1.0)

                while(not rq_get_var("ACT", 1, gripper_socket) == 1 or not rq_get_var("STA", 1, gripper_socket) == 3):
                    sleep(0.1)
                end

                sleep(0.5)
            end
        end
        """
        if not self.is_active():
            self._reset()
            while (not self._get_var(self.ACT) == 0 or not self._get_var(self.STA) == 0):
                time.sleep(0.01)

            self._set_var(self.ACT, 1)
            time.sleep(1.0)
            while (not self._get_var(self.ACT) == 1 or not self._get_var(self.STA) == 3):
                time.sleep(0.01)
        return True
    
    def functionCall(self,func,*args):
        if func == 'auto_calibrate':
            self.auto_calibrate()
        else:
            raise ValueError("Unknown function call? Only support auto_calibrate")

    def sensedPosition(self):
        """Returns the current position as returned by the physical hardware."""
        return [self._get_var(self.POS)/255.0]
    
    def sensedVelocity(self):
        """Returns the current position as returned by the physical hardware."""
        qcur = self._get_var(self.POS)
        qdes = self._get_var(self.PRE)
        if qcur == qdes: sign=0
        elif qdes > qcur: sign=1
        else: sign=-1
        return [sign*self._get_var(self.SPE)/255.0]
    
    def sensedTorque(self):
        """Returns the current position as returned by the physical hardware."""
        qcur = self._get_var(self.POS)
        qdes = self._get_var(self.PRE)
        if qcur == qdes: sign=0
        elif qdes > qcur: sign=1
        else: sign=-1
        return [sign*self._get_var(self.FOR)/255.0]
    
    def commandedPosition(self):
        """Returns the current position as returned by the physical hardware."""
        return [self._get_var(self.PRE)/255.0]
    
    def isMoving(self):
        return RobotiqParallelInterface.ObjectStatus(self._get_var(self.OBJ)) == RobotiqParallelInterface.ObjectStatus.MOVING

    def commandedTorque(self):
        return self._commandedTorque
        # status = RobotiqParallelInterface.ObjectStatus(self._get_var(self.OBJ))
        # if status == RobotiqParallelInterface.ObjectStatus.STOPPED_INNER_OBJECT:
        #     return [1]
        # elif status == RobotiqParallelInterface.ObjectStatus.STOPPED_OUTER_OBJECT:
        #     return [-1]
        # return [0]

    def setPosition(self,q):
        self.setPID(q,[1],[1])
    
    def moveToPosition(self, q, speed):
        return self.setPID(q, [speed], [1])

    def setVelocity(self, dq, ttl=None):
        if ttl is None:
            if dq[0] < 0:
                self.setPID([0],dq)
            else:
                self.setPID([1],dq)
        else:
            q = self.commandedPosition()
            self.setPID([q[0]+ttl*dq[0]],dq)
            
    def setPID(self, q, dq, t=None):
        assert len(q) == 1
        assert len(dq) == 1
        if t is not None:
            assert len(t) == 1
        else:
            t = [1]
        self._commandedTorque = t
        self._move(int(q[0]*255.0),int(abs(dq[0])*255.0),int(abs(t[0])*255.0))

    def close(self):
        if self.socket is not None and self.socket.fileno() != -1:
            print('Robotiq Parallel is disconnecting, have a nice day!')
            self.socket.close()

    def softStop(self):
        # TODO move numbers
        if self.socket.fileno() != -1:
            self.setPID(self.commandedPosition(), [1], [0])


    # WRITE VARIABLES (CAN ALSO READ)
    ACT = 'ACT'  # act : activate (1 while activated, can be reset to clear fault status)
    GTO = 'GTO'  # gto : go to (will perform go to with the actions set in pos, for, spe)
    ATR = 'ATR'  # atr : auto-release (emergency slow move)
    ADR = 'ADR'  # adr : auto-release direction (open(1) or close(0) during auto-release)
    FOR = 'FOR'  # for : force (0-255)
    SPE = 'SPE'  # spe : speed (0-255)
    POS = 'POS'  # pos : position (0-255), 0 = open
    # READ VARIABLES
    STA = 'STA'  # status (0 = is reset, 1 = activating, 3 = active)
    PRE = 'PRE'  # position request (echo of last commanded position)
    OBJ = 'OBJ'  # object detection (0 = moving, 1 = outer grip, 2 = inner grip, 3 = no object at rest)
    FLT = 'FLT'  # fault (0=ok, see manual for errors if not zero)

    ENCODING = 'UTF-8'  # ASCII and UTF-8 both seem to work

    class GripperStatus(Enum):
        """Gripper status reported by the gripper. The integer values have to match what the gripper sends."""
        RESET = 0
        ACTIVATING = 1
        # UNUSED = 2  # This value is currently not used by the gripper firmware
        ACTIVE = 3

    class ObjectStatus(Enum):
        """Object status reported by the gripper. The integer values have to match what the gripper sends."""
        MOVING = 0
        STOPPED_OUTER_OBJECT = 1
        STOPPED_INNER_OBJECT = 2
        AT_DEST = 3

    def _set_vars(self, var_dict):
        """Sends the appropriate command via socket to set the value of n variables, and waits for its 'ack' response.
        :param var_dict: Dictionary of variables to set (variable_name, value).
        :return: True on successful reception of ack, false if no ack was received, indicating the set may not
        have been effective.
        """
        # construct unique command
        cmd = "SET"
        for variable, value in var_dict.items():
            cmd += " {} {}".format(variable,str(value))
        cmd += '\n'  # new line is required for the command to finish
        # atomic commands send/rcv
        with self.command_lock:
            self.socket.sendall(cmd.encode(self.ENCODING))
            data = self.socket.recv(1024)
        return self._is_ack(data)

    def _set_var(self, variable, value):
        """Sends the appropriate command via socket to set the value of a variable, and waits for its 'ack' response.
        :param variable: Variable to set.
        :param value: Value to set for the variable.
        :return: True on successful reception of ack, false if no ack was received, indicating the set may not
        have been effective.
        """
        return self._set_vars(OrderedDict([(variable, value)]))

    def _get_var(self, variable):
        """Sends the appropriate command to retrieve the value of a variable from the gripper, blocking until the
        response is received or the socket times out.
        :param variable: Name of the variable to retrieve.
        :return: Value of the variable as integer.
        """
        # atomic commands send/rcv
        with self.command_lock:
            cmd = "GET {}\n".format(variable)
            self.socket.sendall(cmd.encode(self.ENCODING))
            data = self.socket.recv(1024)

        # expect data of the form 'VAR x', where VAR is an echo of the variable name, and X the value
        # note some special variables (like FLT) may send 2 bytes, instead of an integer. We assume integer here
        var_name, value_str = data.decode(self.ENCODING).split()
        if var_name != variable:
            raise ValueError("Unexpected response {} ({}): does not match '{}'".format(data,data.decode(self.ENCODING),variable))
        try:
            value = int(value_str)
            return value
        except ValueError as e:
            print("Error in parallel return value: ", e)
            print("Got {}".format(value_str))
            raise e

    @staticmethod
    def _is_ack(data):
        return data == b'ack'
    
    def is_active(self):
        """Returns whether the gripper is active."""
        status = self._get_var(self.STA)
        return RobotiqParallelInterface.GripperStatus(status) == RobotiqParallelInterface.GripperStatus.ACTIVE

    def _reset(self):
        """
        Reset the gripper.
        The following code is executed in the corresponding script function
        def rq_reset(gripper_socket="1"):
            rq_set_var("ACT", 0, gripper_socket)
            rq_set_var("ATR", 0, gripper_socket)

            while(not rq_get_var("ACT", 1, gripper_socket) == 0 or not rq_get_var("STA", 1, gripper_socket) == 0):
                rq_set_var("ACT", 0, gripper_socket)
                rq_set_var("ATR", 0, gripper_socket)
                sync()
            end

            sleep(0.5)
        end
        """
        self._set_var(self.ACT, 0)
        self._set_var(self.ATR, 0)
        while (not self._get_var(self.ACT) == 0 or not self._get_var(self.STA) == 0):
            self._set_var(self.ACT, 0)
            self._set_var(self.ATR, 0)
        time.sleep(0.5)

    def auto_calibrate(self, log = True):
        """Attempts to calibrate the open and closed positions, by slowly closing and opening the gripper.
        :param log: Whether to print the results to log.
        """
        # first try to open in case we are holding an object
        (position, status) = self._move_and_wait_for_pos(255, 64, 1)
        if RobotiqParallelInterface.ObjectStatus(status) != RobotiqParallelInterface.ObjectStatus.AT_DEST:
            raise RuntimeError("Calibration failed opening to start: {}".format(status))

        # try to close as far as possible, and record the number
        (position, status) = self._move_and_wait_for_pos(0, 64, 1)
        if RobotiqParallelInterface.ObjectStatus(status) != RobotiqParallelInterface.ObjectStatus.AT_DEST:
            raise RuntimeError("Calibration failed because of an object: {}".format(status))
        assert position <= self._max_position
        self._max_position = position

        # try to open as far as possible, and record the number
        (position, status) = self._move_and_wait_for_pos(255, 64, 1)
        if RobotiqParallelInterface.ObjectStatus(status) != RobotiqParallelInterface.ObjectStatus.AT_DEST:
            raise RuntimeError("Calibration failed because of an object:  {}".format(status))
        assert position >= self._min_position
        self._min_position = position

        if log:
            print("Gripper auto-calibrated to [{}, {}]".format(self._min_position,self._max_position))

        self.properties['joint_limits'] = ([self._min_position/255.0],[self._max_position/255.0])
        
    def _move(self, position, speed, force):
        """Sends commands to start moving towards the given position, with the specified speed and force.
        :param position: Position to move to [min_position, max_position]
        :param speed: Speed to move at [min_speed, max_speed]
        :param force: Force to use [min_force, max_force]
        :return: A tuple with a bool indicating whether the action it was successfully sent, and an integer with
        the actual position that was requested, after being adjusted to the min/max calibrated range.
        """

        def clip_val(min_val, val, max_val):
            return max(min_val, min(val, max_val))

        clip_pos = clip_val(self._min_position, position, self._max_position)
        clip_spe = clip_val(self._min_speed, speed, self._max_speed)
        clip_for = clip_val(self._min_force, force, self._max_force)

        # moves to the given position with the given speed and force
        var_dict = OrderedDict([(self.POS, clip_pos), (self.SPE, clip_spe), (self.FOR, clip_for), (self.GTO, 1)])
        return self._set_vars(var_dict), clip_pos

    def _move_and_wait_for_pos(self, position, speed, force):  # noqa
        """Sends commands to start moving towards the given position, with the specified speed and force, and
        then waits for the move to complete.
        :param position: Position to move to [min_position, max_position]
        :param speed: Speed to move at [min_speed, max_speed]
        :param force: Force to use [min_force, max_force]
        :return: A tuple with an integer representing the last position returned by the gripper after it notified
        that the move had completed, a status indicating how the move ended (see ObjectStatus enum for details). Note
        that it is possible that the position was not reached, if an object was detected during motion.
        """
        set_ok, cmd_pos = self._move(position, speed, force)
        if not set_ok:
            raise RuntimeError("Failed to set variables for move.")

        # wait until the gripper acknowledges that it will try to go to the requested position
        for i in range(400, -1, -1):
            time.sleep(0.001)
            if self._get_var(self.PRE) == cmd_pos:
                break
            if i == 0:
                raise RuntimeError("Gripper command timeout")

        # wait until not moving
        cur_obj = self._get_var(self.OBJ)
        for i in range(4000, -1, -1):
            time.sleep(0.001)
            cur_obj = self._get_var(self.OBJ)
            if RobotiqParallelInterface.ObjectStatus(cur_obj) != RobotiqParallelInterface.ObjectStatus.MOVING:
                break

        # report the actual position and the object status
        final_pos = self._get_var(self.POS)
        final_obj = cur_obj
        return final_pos, RobotiqParallelInterface.ObjectStatus(final_obj)

def make(robotModel, addr='10.1.1.30', port=63352):
    return ThreadedRobotInterface(RobotInterfaceCompleter(RobotiqParallelInterface(addr,port)))


if __name__ == '__main__':
    gripper = RobotiqParallelInterface("10.1.1.30", 63352)
    gripper.initialize()
    gripper.setPID([1], [1], [1])
    gripper.wait()
    gripper.setPID([0], [1], [1])
    gripper.wait()
    gripper.close()
