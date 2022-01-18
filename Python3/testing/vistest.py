from klampt import *
from klampt.model.trajectory import Trajectory,SE3Trajectory,RobotTrajectory
from klampt.vis import editors,colorize
from klampt.math import vectorops,so3,se3
from klampt.io import resource
import time

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *


w = WorldModel()
w.readFile("../../data/tx90cupscupboard.xml")
#w.readFile("../../data/robots/tx90ball.rob")
r = w.robot(0)
q0 = r.getConfig()
r.randomizeConfig()
qrand = r.getConfig()
r.setConfig(q0)
r.randomizeConfig()
qrand2 = r.getConfig()
r.setConfig(q0)

#show edges of a link
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
    while vis.shown():
        time.sleep(5.0)
        vis.scene().setBackgroundImage(None)
        time.sleep(5.0)
        vis.scene().setBackgroundImage(np.asarray(im))
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
    vis.kill()

def test_trajectory_vis():
    #add a "world" item to the scene manager
    vis.add("world",w)
    #show qrand as a ghost configuration in transparent red
    vis.add("qrand",qrand,color=(1,0,0,0.5))
    #show a Trajectory between q0 and qrand
    vis.add("path_to_qrand",RobotTrajectory(r,[0,1],[q0,qrand]))

    #launch the vis loop and window
    vis.show()
    vis.spin(float('inf'))
    vis.kill()

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

def test_dynamic_point_cloud():
    import numpy as np
    from klampt.io import numpy_convert
    w = 640
    h = 480
    #w = 320
    #h = 240
    pc = np.zeros((w*h,3))
    colors = np.zeros((w*h,3))
    x = np.linspace(-1,1,w)
    y = np.linspace(-1,1,h)
    for i in range(h):
        pc[i*w:i*w+w,0] = x
    for i in range(w):
        pc[i:h*w:w,1] = y
    colors[:,0] = pc[:,0]*0.5+0.5
    colors[:,1] = 0
    colors[:,2] = pc[:,1]*0.5+0.5
    tstart = time.time()
    t2last = tstart
    tnumpy_avg = 0
    tklampt_avg = 0
    n = 0
    method = 3
    kpc = PointCloud()
    #structured point clouds are a little slower
    # kpc.setSetting("width",str(w))
    # kpc.setSetting("height",str(h))
    # kpc.setSetting("viewpoint",'0 0 10 0 1 0 0 ')  #looking down
    kpc.propertyNames.resize(3)
    kpc.propertyNames[0] = "r"
    kpc.propertyNames[1] = "g"
    kpc.propertyNames[2] = "b"
    kpc.setPoints(pc)
    kpc.setProperties(colors)
    vis.add('pc',kpc)
    vis.show()
    while vis.shown():
        t0 = time.time()
        pc[:,2] = 0.2*np.sin(2.0*np.dot(pc[:,0:2],[1,1])+(t0-tstart))
        colors[:,1] = (pc[:,2]+0.2)*2.5
        if method==1:
            pc_with_colors = np.hstack((pc,colors))
        t1 = time.time()
        if method == 1:
            kpc = numpy_convert.from_numpy(pc_with_colors,'PointCloud',template=kpc)
        elif method == 2:
            kpc.setPoints(pc)
        elif method == 3: 
            kpc.setPoints(pc)
            kpc.setProperties(colors)
        vis.add('pc',kpc)
        #vis.dirty('pc')
        t2 = time.time()
        tnumpy_avg += 1.0/(n+1)*(t1-t0 - tnumpy_avg)
        tklampt_avg += 1.0/(n+1)*(t2-t1 - tklampt_avg)
        vis.add('nfps','Numpy update: %.2fms'%(1000*tnumpy_avg),position=(10,10),color=(0,0,0,1))
        vis.add('fps','Klampt update: %.2fms'%(1000*tklampt_avg),position=(10,25),color=(0,0,0,1))
        vis.add('drawfps','Overall FPS: %.2f'%(1.0/(t2-t2last)),position=(10,40),color=(0,0,0,1))
        n += 1
        t2last = t2
        time.sleep(0.001)

def test_per_face_colors():
    #shows bug in visualizer with editor
    w = WorldModel()
    w.readFile("../../data/robots/baxter.rob")
    r = w.robot(0)
    for i in range(r.numLinks()):
        colorize.colorize(r.link(i),'z','plasma')
    vis.add('world',w)
    vis.edit(('world',r.getName()))
    vis.show()
    vis.spin(float('inf'))

def test_dynamic_mesh():
    import numpy as np
    from klampt.io import numpy_convert
    res = numpy_convert.to_numpy(r.link(0).geometry(),'Geometry3D')
    T,(verts0,tris) = res
    verts = verts0.copy()
    
    tstart = time.time()
    t2last = tstart
    tnumpy_avg = 0
    tklampt_avg = 0
    n = 0
    method = 1
    kmesh = TriangleMesh()
    #structured point clouds are a little slower
    # kpc.setSetting("width",str(w))
    # kpc.setSetting("height",str(h))
    # kpc.setSetting("viewpoint",'0 0 10 0 1 0 0 ')  #looking down
    kmesh.setVertices(verts0)
    kmesh.setIndices(tris)
    vis.add('mesh',kmesh)
    vis.show()
    while vis.shown():
        t0 = time.time()
        verts[:,0] = verts0[:,0] + 0.2*np.sin(4.0*verts[:,1]+(t0-tstart))
        t1 = time.time()
        if method == 1:
            kmesh = numpy_convert.from_numpy((verts,tris),'TriangleMesh')
        elif method == 2:
            kmesh.setVertices(verts)
        elif method == 3:
            kmesh.setVertices(verts)
            kmesh.setIndices(tris)
        vis.add('mesh',kmesh)
        #vis.dirty('mesh')
        t2 = time.time()
        tnumpy_avg += 1.0/(n+1)*(t1-t0 - tnumpy_avg)
        tklampt_avg += 1.0/(n+1)*(t2-t1 - tklampt_avg)
        vis.add('nfps','Numpy update: %.2fms'%(1000*tnumpy_avg),position=(10,10),color=(0,0,0,1))
        vis.add('fps','Klampt update: %.2fms'%(1000*tklampt_avg),position=(10,25),color=(0,0,0,1))
        vis.add('drawfps','Overall FPS: %.2f'%(1.0/(t2-t2last)),position=(10,40),color=(0,0,0,1))
        n += 1
        t2last = t2
        time.sleep(0.01)

#test_screenshot()
#test_background_image()
#test_basic_plugin()
#test_trajectory_editing()
#test_geometry_editing()
#test_custom_gui()
#test_trajectory_vis()
#test_debug()
test_dynamic_point_cloud()
#test_per_face_colors()
#test_dynamic_mesh()
