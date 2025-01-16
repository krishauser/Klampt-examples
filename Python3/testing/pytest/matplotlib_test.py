import pytest
import time
from klampt import vis
from klampt import *
from klampt.model import sensing
import matplotlib.pyplot as plt
import numpy as np
import random
from OpenGL.GL import *

# Choose one of these methods. 
# - With "nogl", .
# - With "workaround", a tiny GLUT window is popped up, but this messes up the klampt.vis module
# - With "vis", the snapshot call needs to be run in the vis thread using vis.threadCall.
# - With "vis-nothread", the snapshot call is run via vis.loop() in the main thread.
#METHOD = "nogl"
#METHOD = "workaround"
METHOD = "vis"
#METHOD = "vis-nothread"

#Klamp't vis module uses Qt while Matplotlib uses Tk, which don't play well together.
#If this is True, then this tries to alternate between Tk and vis windows
STRESS_TEST_VIS = False

@pytest.fixture(autouse=True)
def world():
    world = WorldModel()
    res = world.readFile('../../../data/simulation_test_worlds/sensortest.xml') # a world with RobotiQ, and a camera
    assert res
    return world

@pytest.fixture(autouse=True)
def sim(world):
    sim = Simulator(world)
    return sim

@pytest.fixture(autouse=True)
def camera(sim):
    camera = sim.controller(0).sensor('rgbd_camera')
    T = ([1,0,0, 0,0,-1,  0,1,0],[0,-2.0,0.5])
    sensing.set_sensor_xform(camera,T,link=-1)
    w = 640
    h = 480
    xfov = np.radians(60)
    fx = w / (2.0 * np.tan(xfov / 2.0))
    yfov = 2.0 * np.arctan(h / (2.0 * fx))
    camera.setSetting("xres",str(w))
    camera.setSetting("yres",str(h))
    camera.setSetting("xfov",str(xfov))
    camera.setSetting("yfov",str(yfov))
    return camera

rgb,depth = None,None

def do_snapshot(world,camera):
    """For the OpenGL operations to succeed, SimRobotSensor.kinematicSimulate
    has to be called within the render loop.
    """
    global rgb,depth
    t0 = time.time()
    #PyQt6: since Matplotlib messes up the GL context and the sensor needs to be simulated in the GL context,
    #we need to make the GL context current before calling the sensor simulation
    if vis.shown():
        vis.scene().window.makeCurrent()
    camera.kinematicReset()
    camera.kinematicSimulate(world,0.01)    
    if vis.shown():
        vis.scene().window.doneCurrent()
    t1 = time.time()
    rgb,depth = sensing.camera_to_images(camera)
    t2 = time.time()
    print("Camera simulated in time %.2fms, %.2fms for download/conversion"%((t2-t0)*1000,(t2-t1)*1000))

    # pc = sensing.image_to_points(depth,rgb,float(camera.getSetting('xfov')),depth_scale=1.0,depth_range=(0.5,5.0),points_format='Geometry3D',all_points=True)
    # pc.setCurrentTransform(*sensing.get_sensor_xform(camera,world.robot(0)))
    # t3 = time.time()
    # print("Camera to point cloud conversion %.2fms"%((t3-t2)*1000))
    # vis.add('point cloud',pc)

    # pc2 = sensing.camera_to_points(camera,points_format='Geometry3D',color_format='rgb')
    # pc2.setCurrentTransform(*sensing.get_sensor_xform(camera,world.robot(0)))
    # t4 = time.time()
    # print("Camera direct to point cloud conversion %.2fms"%((t4-t3)*1000))
    # vis.add('point cloud',pc2)

axs = None
fig = None
closed = False

def show_snapshot_matplotlib(interactive=False):
    """Shows / updates the snapshot in the matplotlib window.  If interactive=True,
    can update the window with new snapshots.  Otherwise, it's a blocking window."""
    global rgb,depth,axs,fig,closed
    if rgb is None or closed:
        return
    if axs is None:
        if interactive:
            plt.ion()
        else:
            plt.ioff()
        fig,axs = plt.subplots(1,2,figsize=(14,4))
        def on_close(event):
            global closed
            closed = True
            if vis.shown():
                vis.show(False)
        if interactive:
            fig.canvas.mpl_connect('close_event', on_close)
            plt.show()
    axs[0].cla()
    axs[1].cla()
    axs[0].imshow(rgb) 
    axs[1].imshow(depth)
    if interactive:
        plt.draw()
        plt.pause(0.001)
    else:
        plt.show()

def close_snapshot_matplotlib():
    """Closes the snapshot window."""
    global axs,fig,closed
    plt.close(fig)
    axs = None
    fig = None
    closed = False

def _test_mpl_nogl(world,camera):
    """There is no attempt to use OpenGL rendering."""
    global closed
    closed = False
    do_snapshot(world,camera)
    show_snapshot_matplotlib()
    close_snapshot_matplotlib()

def _test_gl_workaround(world,camera):
    global closed
    closed = False
    #WORKAROUND FOR OPENGL INITIALIZATION BEFORE simulate
    from OpenGL.GLUT import glutInit,glutDestroyWindow,glutInitDisplayMode,glutInitWindowSize,glutCreateWindow, GLUT_RGB, GLUT_DOUBLE, GLUT_DEPTH, GLUT_MULTISAMPLE

    glutInit ([])
    glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE)
    glutInitWindowSize (1, 1)
    windowID = glutCreateWindow ("test")

    # Default background color
    glClearColor(0.8,0.8,0.9,0)
    # Default light source
    glLightfv(GL_LIGHT0,GL_POSITION,[0,-1,2,0])
    glLightfv(GL_LIGHT0,GL_DIFFUSE,[1,1,1,1])
    glLightfv(GL_LIGHT0,GL_SPECULAR,[1,1,1,1])
    glEnable(GL_LIGHT0)

    glLightfv(GL_LIGHT1,GL_POSITION,[-1,2,1,0])
    glLightfv(GL_LIGHT1,GL_DIFFUSE,[0.5,0.5,0.5,1])
    glLightfv(GL_LIGHT1,GL_SPECULAR,[0.5,0.5,0.5,1])
    glEnable(GL_LIGHT1)

    do_snapshot(world,camera)
    show_snapshot_matplotlib()
    glutDestroyWindow(windowID)
    close_snapshot_matplotlib()
    

def test_vis_multithread(world,camera):
    global closed
    closed = False
    robot = world.robot(0)
    vis.init()
    vis.setBackgroundColor(1,0,0)  #testing red background
    vis.add("world",world)
    vis.add("camera",camera)
    vis.show()
    vis.threadCall(lambda: do_snapshot(world,camera))
    count = 10
    while vis.shown():
        time.sleep(0.25)
        vis.lock()
        q0 = robot.getConfig()
        vmax = robot.getVelocityLimits()
        for i in range(len(q0)):
            q0[i] += random.uniform(-vmax[i],vmax[i])*0.1
        robot.setConfig(q0)
        vis.unlock()
        #vis.threadCall( lambda : show_snapshot_matplotlib(True) )
        show_snapshot_matplotlib(True)
        vis.threadCall( lambda: do_snapshot(world,camera))
        count -= 1
        if count <= 0:
            vis.show(False)
    close_snapshot_matplotlib()
    vis.kill()

def test_vis_singlethread(world,camera):
    global closed
    closed = False
    robot = world.robot(0)
    vis.init()
    vis.setBackgroundColor(1,0,0)  #testing red background
    vis.add("world",world)
    vis.add("camera",camera)
    
    count = 10
    def callback(data = [count]):
        time.sleep(0.25)
        q0 = robot.getConfig()
        vmax = robot.getVelocityLimits()
        for i in range(len(q0)):
            q0[i] += random.uniform(-vmax[i],vmax[i])*0.1
        robot.setConfig(q0)
        do_snapshot(world,camera)
        show_snapshot_matplotlib(True)
        data[0] -= 1
        if data[0] < 0:
            vis.show(False)
    vis.loop(callback=callback)
    close_snapshot_matplotlib()
    vis.kill()

def _test_vis_stress(world,camera):
    #anything but METHOD = "vis" probably won't play nice with this
    robot = world.robot(0)
    vis.init()
    vis.setBackgroundColor(1,0,0)  #testing red background
    vis.add("world",world)
    vis.add("camera",camera)
    vis.show()
    vis.threadCall(lambda: do_snapshot(world,camera))
    while vis.shown():
        time.sleep(0.25)
        vis.lock()
        q0 = robot.getConfig()
        vmax = robot.getVelocityLimits()
        for i in range(len(q0)):
            q0[i] += random.uniform(-vmax[i],vmax[i])*0.1
        robot.setConfig(q0)
        vis.unlock()
        show_snapshot_matplotlib(True)
        vis.threadCall(lambda: do_snapshot(world,camera))
    close_snapshot_matplotlib()
    vis.kill()

    vis.init()
    vis.add('world', world)
    vis.show()
    while vis.shown():
        time.sleep(0.1)

    print("TRYING TO SHOW PLOT AGAIN")
    vis.threadCall(lambda: do_snapshot(world,camera))
    time.sleep(1.0)

    show_snapshot_matplotlib()

    vis.kill()


if __name__ == '__main__':
    world = WorldModel()
    res = world.readFile('../../../data/simulation_test_worlds/sensortest.xml') # a world with RobotiQ, and a camera
    assert res
    sim = Simulator(world)
    camera = sim.controller(0).sensor('rgbd_camera')
    T = ([1,0,0, 0,0,-1,  0,1,0],[0,-2.0,0.5])
    sensing.set_sensor_xform(camera,T,link=-1)
    w = 640
    h = 480
    xfov = np.radians(60)
    fx = w / (2.0 * np.tan(xfov / 2.0))
    yfov = 2.0 * np.arctan(h / (2.0 * fx))
    camera.setSetting("xres",str(w))
    camera.setSetting("yres",str(h))
    camera.setSetting("xfov",str(xfov))
    camera.setSetting("yfov",str(yfov))
    test_vis_multithread(world,camera)