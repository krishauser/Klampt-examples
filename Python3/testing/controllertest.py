from klampt import *
from klampt.control.robotinterfaceutils import RobotInterfaceCompleter,MultiRobotInterface
from klampt.control.robotinterface import RobotInterfaceBase
from klampt.control.simrobotinterface import *
from klampt.control.cartesian_drive import *
from klampt.math import vectorops,so3
from klampt import vis
import math
import time
import csv


def testCompleter():
    w = WorldModel()
    w.readFile("../../data/tx90scenario0.xml")
    r = w.robot(0)
    sim = Simulator(w)
    #TODO: CHANGE ME
    controller = RobotInterfaceCompleter(KinematicSimControlInterface(r))
    #controller = RobotInterfaceCompleter(SimPositionControlInterface(sim.controller(0),sim))
    #controller = RobotInterfaceCompleter(SimMoveToControlInterface(sim.controller(0),sim))
    #controller = RobotInterfaceCompleter(SimVelocityControlInterface(sim.controller(0),sim))
    #controller = RobotInterfaceCompleter(SimFullControlInterface(sim.controller(0),sim))
    testProperties = ['controlRate','parts','sensors','numDOFs','indices']
    testFuncs = ['clock','status','isMoving',
                'sensedPosition','sensedVelocity','sensedTorque','commandedPosition','commandedVelocity','commandedTorque',
                'destinationPosition','destinationVelocity','destinationTime']
    if not controller.initialize():
        raise RuntimeError("There was some problem initializing controller "+str(controller))

    #start logger
    testFile = open('controllertest_results.csv','w',newline='')
    testWriter = csv.writer(testFile)
    testWriter.writerow(testProperties)
    results = []
    for prop in testProperties:
        try:
            results.append(str(getattr(controller,prop)()))
        except Exception as e:
            results.append('Error '+str(e))
    testWriter.writerow(results)
    testWriter.writerow(['emulatorControlMode','baseControlMode']+testFuncs)

    if controller.numDOFs() != r.numDrivers():
        raise RuntimeError("Invalid DOFs")
    if controller.klamptModel() is None:
        raise RuntimeError("Can't get Klampt model")

    q = r.getConfig()[1:]
    print(q)
    print(r.getJointLimits())
    
    controller.setToolCoordinates([0,0,0])
    #TODO: CHANGE ME
    """
    #testing a single movement
    moves = [(1.0,lambda: controller.setPiecewiseLinear([1],[q[:2]+[q[2]+1.0]+q[3:]]))]
    """
    """
    #testing general movements with interruption
    moves = [(1.0,lambda: controller.setPiecewiseLinear([1],[q[:2]+[q[2]+1.0]+q[3:]])),
             (3.0,lambda: controller.setPiecewiseCubic([1],[q],[[0]*len(q)])),
             (3.5,lambda: controller.moveToPosition(q[:2]+[q[2]-1.0]+q[3:])),
             (5.0,lambda: controller.moveToPosition(q,0.1)),
             (8.0,lambda: controller.moveToCartesianPosition((so3.identity(),[0.5,0,1.0]))),
             (10.0,lambda: controller.setCartesianVelocity(([0,0,0],[0,0,1]),3.0)),
             (11.0,lambda: controller.moveToCartesianPosition((so3.identity(),[0.5,0,1.0])))
            ]
    """
    #testing interrupted cartesian velocity movements
    moves = [(0.5,lambda: controller.moveToCartesianPosition((so3.identity(),[0.5,0,1.0]))),
             (2.0,lambda: controller.setCartesianVelocity([0,0,0.1],5.0)) ,
             (3.0,lambda: controller.moveToPosition(q,1))
             #(3.0,lambda: controller.moveToCartesianPosition((so3.identity(),[0.5,0,1.0])))
            ]

    endTime = 13.0
    lastClock = 0
    dt = 1.0/controller.controlRate()
    vis.add("world",w)
    vis.show()
    while controller.status() == 'ok' and vis.shown():  #no error handling done here...
        t0 = time.time()
        vis.lock()
        controller.startStep()
        clock = controller.clock()
        if (clock % 1.0) <= dt:
            controller.print_status()
        for (trigger,callback) in moves:
            if clock > trigger and lastClock <= trigger:
                print("Calling trigger",trigger)
                callback()
        lastClock = clock

        if controller.clock() > endTime:
            vis.unlock()
            break

        controller.endStep()
        #log results to disk
        results = [controller._emulatorControlMode,controller._baseControlMode]
        for func in testFuncs:
            try:
                results.append(str(getattr(controller,func)()))
            except Exception as e:
                results.append('Error '+str(e))
        testWriter.writerow(results)

        if isinstance(controller._base,KinematicSimControlInterface):
            r.setConfig(controller.configToKlampt(controller.commandedPosition()))
        else:
            sim.updateWorld()

        #give visualization some chance to update
        vis.unlock()
        t1 = time.time()
        telapsed = t1 - t0
        time.sleep(max(dt - telapsed,0))
    if vis.shown():
        print("STATUS CHANGED TO",controller.status())
        print("FINAL CLOCK",controller.clock())
        controller.print_status()
    vis.show(False)
    vis.clear()
    testFile.close()


def testCartesianDrive():
    w = WorldModel()
    w.readFile("../../data/tx90scenario0.xml")
    r = w.robot(0)
    solver = CartesianDriveSolver(r)
    #set a non-singular configuration
    q = r.getConfig()
    q[3] = 0.5
    r.setConfig(q)
    solver.start(q,6)
    vis.add("world",w)
    vis.addPlot("timing")
    vis.addPlot("info")
    vis.show()
    time.sleep(0.1)
    dt = 0.01
    t = 0
    while t < 20 and vis.shown():
        vis.lock()
        if t < 2:
            v = [0,0,0.25]
        elif t < 3:
            v = [0,0,-0.1]
        elif t < 3.2:
            v = [0,0,-1]
        elif t < 8:
            v = [0,0,0]
        elif t < 10:
            v = [-1,0,0]
        else:
            v = [1,0,0]
        if t < 4:
            w = [0,0,0]
        elif t < 10:
            w = [0,-0.25,0]
        else:
            w = None
        t0 = time.time()
        progress, qnext = solver.drive(q,w,v,dt)
        t1 = time.time()
        vis.addText("debug","Vel %s"%(str(v),))
        vis.logPlot("timing","t",t1-t0)
        vis.logPlot("info","progress",progress)
        vis.logPlot("info","adj",solver.driveSpeedAdjustment)
        r.setConfig(qnext)
        q = qnext
        vis.unlock()
        vis.add("tgt",solver.driveTransforms[0])
        t += dt
        time.sleep(max(0.005-(t1-t0),0))
    vis.show(False)
    vis.clear()

def testMultiRobot():
    pass


#testCartesianDrive()
testCompleter()
testMultiRobot()
vis.kill()