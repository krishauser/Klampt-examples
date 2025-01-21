
from klampt.math import vectorops,so3,se3
from klampt.sim.simulation import ActuatorEmulator
from klampt.control.simrobotinterface import SimVelocityControlInterface
from klampt import Simulator
import math

def euler_zyx_moments(theta):
    """For the zyx euler angles theta=(rz,ry,rx), produces a matrix A such that
    A*dtheta is the angular velocities when dtheta is the rate of change of the
    euler angles"""
    eu = [0,0,1]
    ev = [0,1,0]
    ew = [1,0,0]
    Ru = so3.rotation([0,0,1],theta[0])
    Rv = so3.rotation([0,1,0],theta[1])
    col1 = eu
    col2 = so3.apply(Ru,ev)
    col3 = so3.apply(Ru,so3.apply(Rv,ew))
    #col1 = [0,0,1]
    #col2 = [c0 -s0 0] [0] = [-s0]
    #       [s0 c0  0]*[1]   [c0 ]
    #       [0  0   1] [0]   [0  ]
    #col3 = Ru*[c1  0 s1] [1] = Ru*[c1 ] = [c1c0]
    #          [0   1 0 ]*[0]      [0  ]   [c1s0]
    #          [-s1 0 c1] [0]      [-s1]   [-s1 ]
    #A = [ 0 -s0 c1c0]
    #    [ 0  c0 c1s0]
    #    [ 1  0  -s1 ]    
    return list(zip(col1,col2,col3))

def euler_zyx_moments_inv(theta):
    """Returns the inverse of the matrix returned by the above procedure"""
    c0 = math.cos(theta[0])
    s0 = math.sin(theta[0])
    c1 = math.cos(theta[1])
    s1 = math.sin(theta[1])
    #A = [ 0 -s0 c1c0]
    #    [ 0  c0 c1s0]
    #    [ 1  0  -s1 ]
    #det(A) = -c1
    #A^-1 = 1/c1*[ s1c0 s0s1 c1  ]
    #            [-c1s0 c1c0 0   ]
    #            [ c0   s0   0   ]
    #A^-1*A = 1/c1*[c1 -s0s1c0+c0s0s1 s1c1c0^2+s1c1s0^2-c1s1 ] = [1 0 0]
    #              [0   c1s0^2+c1c0^2 -c0c1^2s0+s0c1^2c0     ]   [0 1 0]
    #              [0   -s0c0+s0c0    c1c0^2+c1s0^2          ]   [0 0 1]
    sec1 = 1.0/c1
    return [[c0*s1/c1,s0*s1/c1,1],
            [-s0,c0,0],
            [c0/c1,s0/c1,0]]


class SpheroEmulator(ActuatorEmulator):
    def __init__(self,sim,robotIndex = 0):
        self.sim = sim
        self.robotIndex = robotIndex
        self.controller = sim.controller(robotIndex)
        self.robot = sim.world.robot(robotIndex)
        #indices: turn and drive, respectively
        self.velocityLimits = [180*math.pi/180,1080*math.pi/180]
        self.accelLimits = [360*math.pi/180,2080*math.pi/180]
        self.motorSpeeds = [0,0]
        #velocity locking gain
        self.velocityLockGain = 0.01
        #rolling friction
        self.rollingFrictionCoeff = 0.01
        #timestep
        self.dt = 0.01
        self.twist = [0,0,0]
    
    def process(self,commands,dt):
        if commands == None:
            return
        if 'twist' in commands:
            twist = commands['twist']
            assert twist[1] == 0
            self.twist = twist

    def substep(self,dt):
        twist = self.twist
        #compute the angular velocity of the shell in the motor frame
        motorBody = self.sim.body(self.robot.link(5))
        shellBody = self.sim.body(self.robot.link(8))
        motorTwist = motorBody.getVelocity()
        shellTwist = shellBody.getVelocity()
        motorXform = motorBody.getTransform()
        shellXform = shellBody.getTransform()
        shellRelativeAvel = so3.apply(so3.inv(motorXform[0]),vectorops.sub(shellTwist[0],motorTwist[0]))
        #print "Relative angular vel",shellRelativeAvel

        desiredTurnSpeed = twist[2]*self.velocityLimits[0]
        desiredDriveSpeed = 0
        if twist[0] == 0 or twist[0]*self.motorSpeeds[1] < 0: #stop
            desiredDriveSpeed = 0
        else:
            desiredDriveSpeed =  self.motorSpeeds[1]+twist[0]*self.accelLimits[1]*self.dt
        #print "Turn des",desiredTurnSpeed, "drive des",desiredDriveSpeed
        #clamp speeds to limits
        desiredTurnSpeed = max(-self.velocityLimits[0],min(desiredTurnSpeed,self.velocityLimits[0]))
        desiredDriveSpeed = max(-self.velocityLimits[1],min(desiredDriveSpeed,self.velocityLimits[1]))
        terr = desiredTurnSpeed - self.motorSpeeds[0]
        derr = desiredDriveSpeed - self.motorSpeeds[1]
        #clamp desired accelerations to limits
        terr = max(-self.accelLimits[0]*self.dt,min(terr,self.accelLimits[0]*self.dt))
        derr = max(-self.accelLimits[1]*self.dt,min(derr,self.accelLimits[1]*self.dt))
        self.motorSpeeds[0] += terr
        self.motorSpeeds[1] += derr

        #apply locked velocity control to bring shell up to relative speed
        #this is the desired angular velocity of the shell in the motor
        #coordinates
        desiredShellAvel = [self.motorSpeeds[1],0,self.motorSpeeds[0]]
        #print "Desired angular vel",desiredShellAvel
        relativeVelError = vectorops.sub(desiredShellAvel,shellRelativeAvel)
        wrenchlocal = vectorops.mul(relativeVelError,self.velocityLockGain)
        #local wrench is k*(wdes-wrel)
        wrench = so3.apply(motorXform[0],wrenchlocal)
        #print "Wrench to shell",wrench
        motorBody.applyWrench([0,0,0],vectorops.mul(wrench,-1.0))
        shellBody.applyWrench([0,0,0],wrench)
        #disable PID controllers
        self.controller.setTorque([0,0,0])
        
        #apply rolling friction forces
        shellBody.applyWrench([0,0,0],vectorops.mul(shellTwist[0],-self.rollingFrictionCoeff))

def make(sim : Simulator,robotIndex = 0):
    return sim.controller(robotIndex),[SpheroEmulator(sim,robotIndex)]