#!/usr/bin/python

import sys
import math
from klampt import *
from klampt import RobotPoser
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import vis
from klampt.math import vectorops,so3,se3
from klampt.model.subrobot import SubRobotModel
from klampt.model import ik
from klampt.plan import robotplanning
from klampt.vis import editors

class MyGLViewer(GLWidgetPlugin):
    def __init__(self,world):
        GLWidgetPlugin.__init__(self)
        self.world = world
        self.robot = world.robot(0)
        self.subrobots = []
        for i in range(6):
            self.subrobots.append(SubRobotModel(self.robot,list(range(6+i*6,12+i*6))))
        (res,val) = editors.run(editors.SelectionEditor("subrobot",self.subrobots[0]._links,description="sub robot links",world=world,robot=self.robot))
        if res:
            print("Return value",val)
            self.subrobots[0]._links = val
        self.startConfig = self.robot.getConfig()
        self.robotWidget = RobotPoser(world.robot(0))
        self.robotWidget.setActiveDofs(self.subrobots[0]._links)
        self.addWidget(self.robotWidget)

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example prints the config when [space] is pressed
        if c == 'h' or c == '?':
            print("Controls:")
            print("- [space]: print the widget's sub-robot configuration")
            print("- r: randomize the sub-robot configuration")
            print("- p: plan to the widget's sub-robot configuration")
            print("- i: test the IK functions")
            print("- j: test the Jacobian functions")
        elif c == ' ':
            config = self.robotWidget.get()
            subconfig = self.subrobots[0].fromfull(config)
            print("Config:",subconfig)
            self.subrobots[0].setConfig(subconfig)
        elif c == 'r':
            self.subrobots[0].randomizeConfig()
            self.robotWidget.set(self.robot.getConfig())
        elif c == 'p':
            config = self.robotWidget.get()
            subconfig = self.subrobots[0].fromfull(config)
            self.subrobots[0].setConfig(self.subrobots[0].fromfull(self.startConfig))
            settings = { 'type':"sbl", 'perturbationRadius':0.5, 'bidirectional':1, 'shortcut':1, 'restart':1, 'restartTermCond':"{foundSolution:1,maxIters:1000}" }
            plan = robotplanning.planToConfig(self.world,self.subrobots[0],subconfig,
                                  movingSubset='all',
                                  **settings)
            plan.planMore(1000)
            print(plan.getPath())
        elif c == 'i':
            link = self.subrobots[0].link(self.subrobots[0].numLinks()-1)
            print("IK solve for ee to go 10cm upward...")
            obj = ik.objective(link,local=[0,0,0],world=vectorops.add(link.getWorldPosition([0,0,0]),[0,0,0.1]))
            solver = ik.solver(obj)
            res = solver.solve()
            print("  result",res)
            print("  residual",solver.getResidual())
            print(self.robotWidget.set(self.robot.getConfig()))
        elif c == 'j':
            link = self.subrobots[0].link(self.subrobots[0].numLinks()-1)
            print("Jacobian:")
            print(link.getJacobian([0,0,0]))
            print("Position Jacobian:")
            print(link.getPositionJacobian([0,0,0]))
            print("Orientation Jacobian:")
            print(link.getOrientationJacobian())
        else:
            GLWidgetPlugin.keyboardfunc(self,c,x,y)


if __name__ == "__main__":
    print("subrobottest.py: This example demonstrates the subrobot concept")
    
    world = WorldModel()
    res = world.readFile("../../data/athlete_plane.xml")
    if not res:
        raise RuntimeError("Unable to load model "+fn)
    viewer = MyGLViewer(world)
    vis.run(viewer)
