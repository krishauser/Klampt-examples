
import klampt, time
from klampt import vis
from klampt import *
from klampt.model import sensing
import matplotlib.pyplot as plt
import numpy as np
import random

# Choose one of these methods. 
# - With "nogl", there is no attempt to use OpenGL rendering.
# - With "workaround", a tiny GLUT window is popped up, but this messes up the klampt.vis module
# - With "vis", the snapshot call needs to be run in the vis thread using vis.threadCall.
#METHOD = "nogl"
#METHOD = "workaround"
METHOD = "vis"

#Klamp't vis module uses Qt while Matplotlib uses Tk, which don't play well together.
#If this is True, then this tries to alternate between Tk and vis windows
STRESS_TEST_VIS = False

world = WorldModel()
world.readFile('../../data/simulation_test_worlds/sensortest.xml') # a world with RobotiQ, and a camera
sim = Simulator(world)
camera = sim.controller(0).sensor('rgbd_camera')
T = ([1,0,0, 0,0,-1,  0,1,0],[0,-2.0,0.5])
sensing.set_sensor_xform(camera,T,link=-1)
robot = world.robot(0)

rgb,depth = None,None

def do_snapshot():
    global rgb,depth
    camera.kinematicReset()
    camera.kinematicSimulate(world,0.01)
    rgb,depth = sensing.camera_to_images(camera)
    #pc = sensing.image_to_points(depth,rgb,float(camera.getSetting('xfov')),depth_scale=1.0,depth_range=(0.5,5.0),points_format='Geometry3D',all_points=True)
    #pc.setCurrentTransform(*sensing.get_sensor_xform(camera,world.robot(0)))
    #vis.add('point cloud',pc)

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


if METHOD == "nogl":
    do_snapshot()
    show_snapshot_matplotlib()
elif METHOD == "workaround":
    #WORKAROUND FOR OPENGL INITIALIZATION BEFORE simulate
    from OpenGL.GLUT import *
    from OpenGL.GL import *
    glutInit ([])
    glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE)
    glutInitWindowSize (1, 1);
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

    do_snapshot()
    show_snapshot_matplotlib()
else:
    vis.init()
    vis.setBackgroundColor(1,0,0)
    vis.show()
    vis.add("world",world)
    while vis.shown():
        time.sleep(0.25)
        vis.lock()
        #vis.show(False)
        q0 = robot.getConfig()
        vmax = robot.getVelocityLimits()
        for i in range(len(q0)):
            q0[i] += random.uniform(-vmax[i],vmax[i])*0.1
        robot.setConfig(q0)
        vis.unlock()
        show_snapshot_matplotlib(True)

        vis.threadCall( do_snapshot)

    close_snapshot_matplotlib()

    STRESS_TEST_VIS = True
    
if STRESS_TEST_VIS:
    if METHOD == "workaround" or METHOD == "glbackend":
        print("Note: Workaround does something to clobber the Klamp't vis module.")
    #anything but METHOD = "vis" probably won't play nice with this
    vis.add('world', world)
    vis.show()
    while vis.shown():
        time.sleep(0.1)

    if METHOD == 'vis':
        print("Note: vis does something to clobber Tk so that exiting isn't clean.")

    print("TRYING TO SHOW PLOT AGAIN")
    if METHOD == "vis":
        vis.threadCall(do_snapshot)
        time.sleep(1.0)
    else:
        do_snapshot()

    closed = False
    show_snapshot_matplotlib()

    vis.kill()


print("QUITTING")
quit()
