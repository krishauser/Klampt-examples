from klampt import *
from klampt.model.trajectory import Trajectory,SE3Trajectory,RobotTrajectory
from klampt.vis import editors
from klampt.math import vectorops,so3,se3
from klampt.io import resource
import time

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *


w = WorldModel()
#w.readFile("../../data/tx90cupscupboard.xml")
w.readFile("../../data/robots/tx90ball.rob")
r = w.robot(0)
q0 = r.getConfig()
r.randomizeConfig()
qrand = r.getConfig()
r.setConfig(q0)
r.randomizeConfig()
qrand2 = r.getConfig()
r.setConfig(q0)

r.link(0).appearance().setDraw(Appearance.EDGES,True)
r.link(0).appearance().setColor(Appearance.EDGES,1,1,1,0.2)

#vis.init('PyQt')
#vis.init('GLUT')

def test_screenshot():
    import numpy as np
    from PIL import Image,ImageMath
    vis.add("world",w)
    #vis.setColor(vis.getItemName(w.robot(0)),0.5,0.5,0.5,0.5)
    def take_screenshot1():
        print("Saving as temp1.jpg, temp2.png")
        screenshot1,screenshot2 = vis.screenshot('Image',True)
        screenshot1.save("temp1.jpg","JPEG")
        scaled = ImageMath.eval("a/8*255",a=screenshot2)
        print("min/max",scaled.getextrema())
        scaled = scaled.convert("L")
        print("Converted to L: min/max",scaled.getextrema())
        scaled.save("temp2.png","PNG")
    def take_screenshot2():
        print("Saving as temp3.jpg, temp4.png")
        screenshot1,screenshot2 = vis.screenshot('numpy',True)
        screenshot1 = Image.fromarray(screenshot1)
        screenshot1.save("temp3.jpg","JPEG")
        scaled = (screenshot2/8*255).clip(0,255)
        print("min",scaled.min(),"max",scaled.max())
        im = Image.fromarray(scaled.astype(np.uint8))
        print("Converted to Image: min/max",im.getextrema())
        im.save("temp4.png","PNG")
    def take_screenshot3():
        print("Showing as Matplotlib plots")
        from matplotlib import pyplot as plt
        screenshot1,screenshot2 = vis.screenshot('numpy',True)
        fig,axs = plt.subplots(1,2,figsize=(14,4))
        axs[0].imshow(screenshot1)
        axs[1].imshow(screenshot2)
        plt.show()
            
    vis.addAction(take_screenshot1,'Screenshot Image','1')
    vis.addAction(take_screenshot2,'Screenshot numpy','2')
    vis.addAction(take_screenshot3,'Screenshot numpy + matplotlib','3')
    
    vis.show()

    #testing initial screen shot
    screenshot = vis.screenshot('Image')
    screenshot.save("temp0.jpg","JPEG")
    vis.spin(float('inf'))
    vis.kill()

def test_background_image():
    import numpy as np
    from PIL import Image
    im = Image.open("../../data/simulation_test_worlds/hauser.bmp")
    vis.add("world",w)
    vis.resizeWindow(im.width,im.height)
    vis.setColor(vis.getItemName(w.robot(0)),0.5,0.5,0.5,0.5)
    vis.show()
    vis.scene().setBackgroundImage(np.asarray(im))
    vis.spin(float('inf'))
    vis.kill()
    

def test_basic_plugin():
    from klampt.vis.glprogram import GLNavigationProgram
    r.setConfig(qrand)

    class KBTest(GLNavigationProgram):
        def __init__(self):
            GLNavigationProgram.__init__(self,'KBTest')
        def keyboardfunc(self,c,x,y):
            print(c,x,y)
    vis.run(KBTest())

def test_trajectory_editing():
    traj = SE3Trajectory([0,1],[se3.identity(),se3.identity()])
    saved,result = resource.edit("se3 traj",traj,'SE3Trajectory','Some spatial trajectory',world=w,referenceObject=r.link(7))

    traj = Trajectory([0,1],[[0,0,0],[0,0,1]])
    saved,result = resource.edit("point traj",traj,'auto','Some point trajectory',world=w,referenceObject=r.link(7))

    traj = RobotTrajectory(r,[0,1,2],[q0,qrand,qrand2])
    saved,result = resource.edit("robot traj",traj,'auto','Random robot trajectory',world=w,referenceObject=r)
    vis.kill()

def test_geometry_editing():
    g = GeometricPrimitive()
    g.setPoint([-0.5,0,1])
    saved,result = resource.edit("Point",g,world=w)
    print(result)
    g.setSphere([-0.5,0,1],0.3)
    saved,result = resource.edit("Sphere",g,world=w)
    print(result)
    g.setAABB([-0.5,0,1],[-0.2,0.1,1.3])
    saved,result = resource.edit("AABB",g,world=w)
    print(result)
    g.setBox([-0.5,0,1],so3.identity(),[0.4,0.4,0.4])
    saved,result = resource.edit("Box",g,world=w)
    print(result)
    vis.kill()

def test_custom_gui():

    def make_gui(glwidget):
        #place your Qt code here and place the glwidget where it needs to be
        w = QMainWindow()
        w.resize(800,800)
        glwidget.setMaximumSize(4000,4000)
        glwidget.setSizePolicy(QSizePolicy(QSizePolicy.Maximum,QSizePolicy.Maximum))
        area = QWidget(w)
        layout = QVBoxLayout()
        layout.addWidget(glwidget)
        layout.addWidget(QPushButton("Click me"))
        area.setLayout(layout)
        w.setCentralWidget(area)
        return w

    vis.customUI(make_gui)
    vis.add("world",w)
    vis.show()
    vis.spin(float('inf'))

def test_trajectory_vis():
    #add a "world" item to the scene manager
    vis.add("world",w)
    #show qrand as a ghost configuration in transparent red
    vis.add("qrand",qrand,color=(1,0,0,0.5))
    #show a Trajectory between q0 and qrand
    vis.add("path_to_qrand",RobotTrajectory(r,[0,1],[q0,qrand]))

    #launch the vis loop and window
    vis.show()
    while vis.shown():
        time.sleep(0.01)

def test_debug():
    vis.debug(w.robot(0),centerCamera=True)
    g = Geometry3D()
    g.loadFile("../../data/objects/srimugsmooth.off")
    vis.debug(g,centerCamera=True)

    pt = [0,0,2]
    traj = Trajectory([0,1,2],[[0,0,2],[1,0,2],[1,1,2]])
    vis.debug('qrand',[qrand,qrand2,q0],{'color':[1,0,0,0.5]},pt,world=w)
    vis.debug('qrand',qrand,{'color':[1,0,0,0.5]},pt,world=w,centerCamera=r.link(6))
    vis.debug('qrand',qrand,{'color':[1,0,0,0.5]},world=w,followCamera=r.link(6))
    vis.debug('qrand',qrand,{'color':[1,0,0,0.5]},pt=pt,world=w,animation=traj)
    vis.debug('qrand',qrand,{'color':[1,0,0,0.5]},pt=pt,world=w,centerCamera='pt')
    vis.debug('qrand',qrand,{'color':[1,0,0,0.5]},pt=pt,world=w,animation=traj,followCamera='pt')

    milestones = []
    for i in range(5):
        r.randomizeConfig()
        milestones.append(r.getConfig())
    r.setConfig(q0)
    qtraj = RobotTrajectory(r,[0,1,2,3,4],milestones)
    vis.debug('qrand',qrand,{'color':[1,0,0,0.5]},world=w,animation=qtraj)
    #this doesn't work -- qrand is not being tracked
    vis.debug('qrand',qrand,{'color':[1,0,0,0.5]},world=w,animation=qtraj,followCamera=r.link(6))

test_screenshot()
#test_background_image()
#test_debug()