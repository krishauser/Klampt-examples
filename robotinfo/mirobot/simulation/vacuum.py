from klampt.sim.simulation import SensorEmulator,ActuatorEmulator
from klampt.math import vectorops,so3,se3
from klampt.model import collide
import math

class VacuumSensor(SensorEmulator):
    def __init__(self):
        self.flow = 0
    def update(self):
        return {'flow':self.flow}

class VacuumEmulator(ActuatorEmulator):
    def __init__(self,sim,robotIndex,linkIndex,position,direction,strength,radius,compliance=0,falloffCoefficient=100,flowSensor:VacuumSensor=None):
        self.sim = sim
        self.collider = collide.WorldCollider(sim.world)
        self.name = 'vacuum'
        self.robot = sim.world.robot(robotIndex)
        self.link = self.robot.link(linkIndex)
        self.collider.ignoreCollision(self.link)
        self.position = position
        self.direction = direction
        self.strength = strength
        self.radius = radius
        self.compliance = compliance
        self.falloffCoefficient = falloffCoefficient
        self.currentCommand = 0
        self.flowSensor = flowSensor
    
    def process(self, commands : dict, dt : float) -> None:
        self.currentCommand = commands.get(self.name,0)
        if self.name in commands:
            del commands[self.name]

    def substep(self, dt : float) -> None:
        if self.currentCommand == 0: 
            if self.flowSensor is not None:
                self.flowSensor.flow = 0
            return
        #determine suction
        pworld = self.link.getWorldPosition(self.position)
        dworld = self.link.getWorldDirection(self.direction)
        Tvacuum = (so3.canonical(dworld),pworld)
        points = []
        for i in range(16):
            theta = i/16*math.pi*2
            points.append(se3.apply(Tvacuum,[0,self.radius*math.cos(theta),self.radius*math.sin(theta)]))
        fractionCovered = 0
        sumStrength = 0
        for i,p in enumerate(points):
            res = self.collider.rayCast(p,dworld)
            if res is None:
                continue
            worldBody,point = res
            d = max(vectorops.distance(point,p)-self.compliance,0)
            flowResistance = math.exp(-d*self.falloffCoefficient)
            fractionCovered += flowResistance
            strength = self.strength*self.currentCommand*flowResistance/len(points)
            sumStrength += strength
            centerDirection = vectorops.unit(vectorops.sub(point,pworld))
            self.sim.body(self.link).applyForceAtPoint(vectorops.mul(centerDirection,strength),p)
            self.sim.body(worldBody).applyForceAtPoint(vectorops.mul(centerDirection,-strength),point)
            #TODO: slopes
        fractionCovered /= len(points)
        if self.flowSensor is not None:
            self.flowSensor.flow = (1.0-fractionCovered)

    def drawGL(self) -> None:
        from OpenGL import GL
        pworld = self.link.getWorldPosition(self.position)
        dworld = self.link.getWorldDirection(self.direction)
        Tvacuum = (so3.canonical(dworld),pworld)
        points = []
        for i in range(16):
            theta = i/16*math.pi*2
            points.append(se3.apply(Tvacuum,[0,self.radius*math.cos(theta),self.radius*math.sin(theta)]))
        GL.glDisable(GL.GL_LIGHTING)
        GL.glColor3f(1,1,0)
        GL.glBegin(GL.GL_POINTS)
        GL.glVertex3fv(pworld)
        GL.glEnd()
        GL.glBegin(GL.GL_LINE_LOOP)
        GL.glColor3f(1.0,1.0-self.currentCommand,0)
        for p in points:
            GL.glVertex3fv(p)
        GL.glEnd()
        GL.glEnable(GL.GL_LIGHTING)
        return

def make(sim,robotIndex):
    flowSensor = VacuumSensor()
    return lambda controller:None,[flowSensor,VacuumEmulator(sim,robotIndex,7,[0,0,-0.0300],[0,0,-1],0.25*9.8,0.007,0.002,flowSensor=flowSensor)]
