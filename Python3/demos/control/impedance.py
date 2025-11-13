from klampt.control import RobotInterfaceBase
from klampt.control.robotinterfaceutils import RobotInterfaceCompleter
from klampt import RobotModel
from typing import Union,Optional
from klampt.math import so3, se3
from klampt.math import vectorops as vo
from dataclasses import dataclass,field
from collections import deque
import numpy as np
import copy
from klampt.model.typing import RigidTransform,Vector3
from typing import Tuple, List


@dataclass
class ImpedanceControlSettings:
    stiffness : Union[float,List[float],np.ndarray]
    mass : Union[float,List[float],np.ndarray] = 1.0
    damping : Optional[Union[float,List[float],np.ndarray]] = None   #None indicates critical damping
    wrench_sensor : str = 'wrench'   # names a sensor in the robot interface. Usually 'wrench' or 'filtered_wrench'
    wrench_sensor_frame : Union[int,str] = 0 # names the link in whose coordinate frame the wrench is reported. Default is the robot's base link.  For a simulated sensor, this should be set to the sensor link
    wrench_sensor_ee_xform : RigidTransform = field(default_factory = lambda : se3.identity())  # local transform of the sensor frame to the end effector frame
    tool_center : Vector3 = field(default_factory = lambda : [0,0,0])   # local position of the tool center in the end effector frame
    max_trans_velocity : float = 3   # m/s
    max_rot_velocity : float = 3     # rad/s
    max_trans_accel : float = 5      # m/s^2
    max_rot_accel : float = 20       # rad/s^2
    feedforward_gain : float = 1.0   # 0.0 means no feedforward, 1.0 means full feedforward
    simulation_substeps : int = 1



class ImpedanceController(RobotInterfaceCompleter):
    """A basic impedance controller that can be used with a RobotInterfaceBase.

    Make sure to set the wrench sensor ID and frame correctly.  Then, tune the 
    stiffness and damping parameters to get the desired behavior.

    The mass is simulated about the tool center point.
    """
    def __init__(self, base_interface : RobotInterfaceBase, settings : ImpedanceControlSettings, base_initialized=False):
        super().__init__(base_interface, base_initialized)
        self.settings = settings
        self.dt = None
        self._klampt_model = None
        self.impedance_active = False
        self.impedance_mode = 'position'
        self.velocity_ttl = None
        self.steps_since_target_set = 0
        self.steps_since_impedance_activation = 0
        self.T_mass = None
        self.x_dot_mass = None
        self.T_g = None  #target
        self.x_dot_g = ([0]*3,[0]*3)  #target velocity
        self.T_g_prev = None # previous target, used for differencing
        self.old_tool_coordinates = None

        self.set_stiffness(settings.stiffness)
        self.set_mass(settings.mass)
        self.set_damping(settings.damping)
    
    def __str__(self):
        return 'Impedance({})'.format(self._base)
    
    def set_stiffness(self, K : Union[int,float,list,tuple,np.ndarray]):
        """Sets the stiffness to a diagonal or 6x6 matrix"""
        if isinstance(K, (int, float)):
            K = np.eye(6)*K
        elif isinstance(K, (list, tuple)):
            K = np.diag(K)
        assert K.shape == (6,6)
        self.K = K
        self.Kinv = np.linalg.inv(K)

    def set_mass(self, M : Union[int,float,list,tuple,np.ndarray]):
        """Sets the mass to a diagonal or 6x6 matrix"""
        if isinstance(M, (int, float)):
            M = np.eye(6)*M
        elif isinstance(M, (list, tuple)):
            M = np.diag(M)
        assert M.shape == (6,6)
        self.M = M
        self.Minv = np.linalg.inv(self.M)

    def set_damping(self, B : Union[None,int,float,list,tuple,np.ndarray], auto_fudge_factor = 2.0):
        """If B is None, sets damping automatically to be critically damped.
        
        The fudge factor is used to improve stability in the case of robot / simulator
        instability.
        """
        if B is None:
            self.B = 2.0*np.sqrt(np.dot(self.K,self.M)) * auto_fudge_factor
        else:
            if isinstance(B, (int, float)):
                B = np.eye(6)*B
            elif isinstance(B, (list, tuple)):
                B = np.diag(B)
            self.B = B
        assert self.B.shape == (6,6)
        
    def initialize(self):
        if not super().initialize():
            return False
        if self.settings.wrench_sensor not in self.sensors():
            print("ImpedanceController: Wrench sensor",self.settings.wrench_sensor,"not found in sensors",self.sensors())
            return False 
        self.dt = 1.0 / self._base.controlRate()
        self._klampt_model = super().klamptModel()
        if self._klampt_model is None:
            print("ImpedanceController: Could not get klampt model from base interface")
            return False
        ee_link = self._klampt_model.driver(self._klampt_model.numDrivers()-1).getAffectedLink()
        self.end_effector_link = self._klampt_model.link(ee_link)
        self.wrench_sensor_link = self._klampt_model.link(self.settings.wrench_sensor_frame)
        return True
    
    def setControlMode(self, mode : str):
        if mode == 'impedance':
            self.impedance_active = True
            self.impedance_mode = None
            self.old_tool_coordinates = self.getToolCoordinates()
            self.setToolCoordinates(self.settings.tool_center)
        else:
            self.deactivateImpedance()
            return super().setControlMode(mode)
    
    def setPosition(self, q):
        self.deactivateImpedance()
        return super().setPosition(q)

    def setVelocity(self, q):
        self.deactivateImpedance()
        return super().setPosition(q)

    def state(self):
        """Return some state variables, used mostly for debugging"""
        return {'T_mass':self.T_mass,'x_dot_mass':self.x_dot_mass}

    def deactivateImpedance(self):
        self.impedance_active = False
        self.T_mass = None
        self.x_dot_mass = None
        self.T_g = None 
        self.T_g_prev = None
        self.steps_since_target_set = 0
        self.steps_since_impedance_activation = 0
        if self.old_tool_coordinates is not None:
            self.setCartesianPosition(self.old_tool_coordinates)
            self.old_tool_coordinates = None

    def get_EE_transform(self, tool_center = None) -> RigidTransform:
        if tool_center is None:
            tool_center = self.settings.tool_center
        self._klampt_model.setConfig(self._klampt_model.configFromDrivers(self.commandedPosition()))
        T_ee = self.end_effector_link.getTransform()
        return (T_ee[0],se3.apply(T_ee,tool_center))

    def get_EE_velocity(self, tool_center = None) -> Tuple[Vector3,Vector3]:
        """Returns the 6D vector of the tool's velocity, consisting of
        (angular velocity,linear velocity)"""
        if tool_center is None:
            tool_center = self.settings.tool_center
        self._klampt_model.setConfig(self._klampt_model.configFromDrivers(self.commandedPosition()))
        self._klampt_model.setVelocity(self._klampt_model.velocityFromDrivers(self.commandedVelocity()))
        v = self.end_effector_link.getPointVelocity(tool_center)
        w = self.end_effector_link.getAngularVelocity()
        return (w,v)

    def get_EE_jacobian(self, tool_center = None) -> np.ndarray:
        """Returns 6xn matrix"""
        if tool_center is None:
            tool_center = self.settings.tool_center
        return self.end_effector_link.getJacobian(tool_center)

    def get_EE_wrench(self) -> List[float]:
        return self.wrench_at_EE(self.sensorMeasurements(self.settings.wrench_sensor))

    def wrench_at_EE(self, wrench_at_sensor : List[float]) -> List[float]:
        """Transforms a wrench in the sensor frame to a world frame wrench centered
        at the tool position.

        Wrench is a 6D vector stacking [force, torque].
        """
        self._klampt_model.setConfig(self._klampt_model.configFromDrivers(self.commandedPosition()))
        if self.wrench_sensor_link.getParent()  == self.end_effector_link.index:  #readings are reported in frame attached to end effector
            sensor_frame_world = self.wrench_sensor_link.getTransform()
            sensor_orientation = sensor_frame_world[0]
            sensor_translation = se3.mul(self.wrench_sensor_link.getTransform(),self.settings.wrench_sensor_ee_xform)[1]
        else:
            sensor_frame_world = se3.mul(self.wrench_sensor_link.getTransform(),self.settings.wrench_sensor_ee_xform)
            sensor_orientation = sensor_frame_world[0]
            sensor_translation = sensor_frame_world[1]
        f_sensor = wrench_at_sensor[:3]
        m_sensor = wrench_at_sensor[3:]
        f_world = so3.apply(sensor_orientation,f_sensor)
        m_world = so3.apply(sensor_orientation,m_sensor)
        moment_arm = vo.sub(self.end_effector_link.getWorldPosition(self.settings.tool_center),sensor_translation)
        m_tool = vo.sub(m_world,vo.cross(moment_arm,f_world))
        return list(f_world) + list(m_tool)

    def setCartesianPosition(self, transform : RigidTransform, frame : str = 'world'):
        if self.impedance_active:
            self.steps_since_target_set = 0
            if self.T_g is None:  # First time setting target
                self.T_mass = self.get_EE_transform()
                self.x_dot_mass = self.get_EE_velocity()
                self.T_g_prev = transform
            self.T_g = copy.copy(transform) 
            self.x_dot_g = ([0]*3,[0]*3)
            self.impedance_mode = 'position'
        else:
            self.steps_since_target_set = 0
            self.steps_since_impedance_activation = 0
            super().setCartesianPosition(transform, frame)

    def setCartesianVelocity(self, dxparams : Tuple[Vector3,Vector3], 
            ttl: Optional[float] = None,
            frame : str = 'world'):
        if self.impedance_active:
            self.steps_since_target_set = 0
            if self.T_g is None:  # First time setting target
                self.T_mass = self.get_EE_transform()
                self.x_dot_mass = self.get_EE_velocity()
                self.T_g_prev = self.T_mass
                self.T_g = self.T_mass
            self.x_dot_g = (copy.copy(dxparams[0]),copy.copy(dxparams[1]))
            self.velocity_ttl = ttl
            self.impedance_mode = 'velocity'
        else:
            self.steps_since_target_set = 0
            self.steps_since_impedance_activation = 0
            super().setCartesianVelocity(dxparams, ttl, frame)

    def endStep(self):
        if self.impedance_active:
            self.steps_since_target_set += 1
            self.steps_since_impedance_activation += 1
            if self.impedance_mode == 'position':
                dT = se3.error(self.T_g,self.T_g_prev) 
                self.x_dot_g = (vo.div(dT[0:3],self.dt),vo.div(dT[3:6],self.dt))
                T = self._advanceImpedanceController(self.T_g, self.x_dot_g)
                self.T_g_prev = self.T_g
                super().setCartesianPosition(T)
            elif self.impedance_mode == 'velocity':
                T_g_prev = self.T_g
                if self.velocity_ttl is None or self.velocity_ttl > 0:
                    advance = self.dt
                    if self.velocity_ttl is not None:
                        advance = min(advance,self.velocity_ttl)
                    dw = vo.mul(self.x_dot_g[0],advance)
                    dv = vo.mul(self.x_dot_g[1],advance)
                    self.T_g = se3.mul((so3.from_rotation_vector(dw),dv),self.T_g_prev)
                    if self.velocity_ttl is not None:
                        self.velocity_ttl -= self.dt
                T = self._advanceImpedanceController(self.T_g, self.x_dot_g)
                self.T_g_prev = T_g_prev
                super().setCartesianPosition(T)
            elif self.impedance_mode is None:
                pass
            else:
                raise NotImplementedError("Invalid impedance_mode?")
        super().endStep()

    def _advanceImpedanceController(self, target, target_velocity):
        """Can be overridden to provide a different impedance control method."""
        wrench = self.wrench_at_EE(self.sensorMeasurements(self.settings.wrench_sensor))
        current_pose = self.get_EE_transform()

        #Use the target velocity as a feedforward term
        if self.settings.feedforward_gain > 0:
            w,v = target_velocity
            w = vo.mul(w,self.settings.feedforward_gain*self.dt)
            v = vo.mul(v,self.settings.feedforward_gain*self.dt)
            self.T_mass = se3.mul((so3.from_moment(w),v),self.T_mass)

        #compute feedback term
        N = self.settings.simulation_substeps # tunable
        for i in range(N):
            self.T_mass, self.x_dot_mass = self._simulateMassSpringDamper(wrench = wrench, m_inv = self.Minv,\
                K = self.K,B = self.B,T_curr = self.T_mass,x_dot_curr = self.x_dot_mass,\
                T_g = target, x_dot_g = ([0]*3,[0]*3), dt = self.dt/N)
        T = self.T_mass
        return T

    def _simulateMassSpringDamper(self,wrench,m_inv,K,B,T_curr,x_dot_curr,T_g,x_dot_g,dt) -> Tuple[RigidTransform, Tuple[Vector3,Vector3]]:
        """
        Simulate a mass spring damper under external load, semi-implicit Euler integration,
        and velocity / acceleration limits.

        Parameters:
        -----------------
        m_inv: 6x6 numpy array, inverse of the mass matrix
        K: 6x6 numpy array, spring constant matrix
        B, 6x6 numpy array, damping constant matrix
        T_curr: rigid transform (R,t), current transform of the mass
        x_dot_curr: pair (w,v), current speed
        T_g: rigid transform (R,t), target transform
        x_dot_g: a pair (w,v), target velocity
        dt:simulation dt

        Returns:
        -----------------
        x,v: transform, velocity (np array of 6)
        """
        e = se3.error(T_g,T_curr)
        e = np.array(e[3:6] + e[0:3])

        x_dot_g = np.array(x_dot_g[1] + x_dot_g[0])
        v = np.array(x_dot_curr[1] + x_dot_curr[0])
        e_dot = x_dot_g - v
        wrench_total = wrench + np.dot(K,e) + np.dot(B,e_dot)
        
        a = np.dot(m_inv,wrench_total)
        #limit maximum acceleration
        a = np.clip(a,[-self.settings.max_trans_accel]*3 + [-self.settings.max_rot_accel]*3,[self.settings.max_trans_accel]*3 + [self.settings.max_rot_accel]*3)
        v = v + a*dt

        #limit maximum velocity
        v = np.clip(v,[-self.settings.max_trans_velocity]*3+[-self.settings.max_rot_velocity]*3,
            [self.settings.max_trans_velocity]*3+[self.settings.max_rot_velocity]*3)
        dx = v*dt
        # T = se3.mul((so3.from_moment(dx[3:6]),dx[0:3]),T_curr)

        #Using angle impedance
        T = (so3.mul(so3.from_moment(dx[3:6]),T_curr[0]),vo.add(T_curr[1], dx[0:3]))
        # Not using angle impedance
        #T = (T_g[0],vo.add(T_curr[1], dx[0:3]))         
        return T,(v.tolist()[3:],v.tolist()[:3])



@dataclass
class AdaptiveImpedanceControlSettings:
    force_min : float
    force_max : float
    torque_min : float
    torque_max : float


class AdaptiveImpedanceController(ImpedanceController):
    """Adapted from Noah Franschescini's method proposed in ICRA2025 submission.

    Evolves an attractor point toward the target pose based on the sensed wrench.
    The attractor point guides the mass-spring-damper system.
    """
    def __init__(self, base_interface : RobotInterfaceBase,
                 settings : ImpedanceControlSettings,
                 adaptive_settings : AdaptiveImpedanceControlSettings,
                 base_initialized=False):
        super().__init__(base_interface, settings, base_initialized)
        self.adaptive_settings = adaptive_settings
        self.T_attractor = None
        self.damping_alpha = None
        self.damping_beta = None

    def deactivateImpedance(self):
        self.T_attractor = None
        return super().deactivateImpedance()

    def state(self):
        """Return some state variables, used mostly for debugging"""
        istate = super().state()
        istate['T_attractor'] = self.T_attractor
        istate['damping_alpha'] = self.damping_alpha
        istate['damping_beta'] = self.damping_beta
        return istate

    def _advanceImpedanceController(self, target, target_velocity):
        """
        Uses an attraction point for impedance control and saturates it depending on
        the sensed wrench.
        """
        wrench = self.wrench_at_EE(self.sensorMeasurements(self.settings.wrench_sensor))
        current_pose = self.get_EE_transform()
        if self.T_attractor is None:
            self.T_attractor = current_pose

        #Use the target velocity as a feedforward term
        if self.settings.feedforward_gain > 0:
            w,v = target_velocity
            w = vo.mul(w,self.settings.feedforward_gain*self.dt)
            v = vo.mul(v,self.settings.feedforward_gain*self.dt)
            self.T_mass = se3.mul((so3.from_moment(w),v),self.T_mass)

        T_target_dot = se3.error(target, self.T_g_prev)[-3:]
        R_target_prev, t_prev = self.T_g_prev
        R_target_curr, t_target_curr = target
        w_target_prev_to_curr = so3.rotation_vector(so3.mul(so3.inv(R_target_curr),R_target_prev))
        ff_diff = T_target_dot + w_target_prev_to_curr

        #scale damping value for attractor point by the sensed wrench
        def get_dampened_values(wmins, wmaxs, wrench, values):
            damp = 6*[0]
            damped_vals = 6*[0]
            for i,curr_wrench in enumerate(wrench):
                if curr_wrench*values[i] < 0:
                    damp[i] = min(1,max(1-(abs(curr_wrench)-wmins[i])/(wmaxs[i]-wmins[i]),0))
                    damped_vals[i] = damp[i]*values[i]
                else:  #no damping
                    damp[i] = 1.0
                    damped_vals[i] = values[i]
            return damp,damped_vals
        
        #dampen values based on wrench felt
        linear_min = self.adaptive_settings.force_min
        rotational_min = self.adaptive_settings.torque_min
        linear_max = self.adaptive_settings.force_max
        rotational_max = self.adaptive_settings.torque_max
        wmins = 3*[linear_min] + 3*[rotational_min]
        wmaxs = 3*[linear_max] + 3*[rotational_max]
        damp,so3_dot = get_dampened_values(wmins,wmaxs,wrench,ff_diff)
        self.damping_alpha = damp
        
        #ignore feedforward deltas that are moving towards attractor point

        prev_planned_rot_vector = so3.rotation_vector(self.T_g_prev[0])
        prev_planned_vectorized = self.T_g_prev[1] + prev_planned_rot_vector

        curr_rot_vector = so3.rotation_vector(self.T_attractor[0])
        curr_pose_vectorized = self.T_attractor[1] + curr_rot_vector

        diff_poses = vo.sub(curr_pose_vectorized,prev_planned_vectorized)

        dot = vo.dot(so3_dot,diff_poses)
        if dot > 0:
            #project so3_dot so its dot product is 0 with diff_poses
            so3_dot = vo.sub(so3_dot,vo.div(vo.mul(diff_poses,float(dot)),vo.dot(diff_poses,diff_poses)))

        #if a value is positive, ignore that portion of the update
        #since that means planned point is moving towards attractor
        # dots = vo.mul(so3_dot,diff_poses)
        # for i,val in enumerate(dots):
        #     if val > 0:
        #         so3_dot[i] = 0
        
        #scale down velocity terms
        dt_diff = se3.error(target, self.T_mass)
        lie_deriv = vo.mul(dt_diff[3:6] + dt_diff[0:3], self.dt)
        #lie_deriv = vo.mul(lie_deriv,0.1) KH: ??

        damp,so3_vel_add = get_dampened_values(wmins,wmaxs,wrench, lie_deriv)
        self.damping_beta = damp

        #add velocity to current dt
        vo.add(so3_dot,so3_vel_add)

        #update attractor based on forces sensed from EE
        #update attractor SE3
        R_updated_attraction = so3.mul(so3.from_rotation_vector(so3_dot[3:6]),self.T_attractor[0])
        T_updated_attraction = vo.add(self.T_attractor[1],so3_dot[0:3])

        self.T_attractor = (R_updated_attraction, T_updated_attraction)
        #simulate spring-damper system between attractor point and current position
        N = self.settings.simulation_substeps # tunable
        for i in range(N):
            self.T_mass, self.x_dot_mass = self._simulateMassSpringDamper(wrench = wrench, m_inv = self.Minv,\
                K = self.K,B = self.B,T_curr = self.T_mass,x_dot_curr = self.x_dot_mass,\
                T_g = self.T_attractor, x_dot_g = ([0]*3,[0]*3),dt = self.dt/N)

        T = self.T_mass
        return T
    
