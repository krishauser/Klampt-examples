
import klampt, time
from klampt import vis
from klampt import *
from klampt.model import sensing
import matplotlib.pyplot as plt
import numpy as np

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

def do_snapshot():
    camera.kinematicReset()
    camera.kinematicSimulate(world,0.01)
    rgb,depth = sensing.camera_to_images(camera)
    print("RGB shape:",rgb.shape)
    print("Depth shape:",depth.shape)
    measurements = camera.getMeasurements()
    plt.imshow(rgb) 
    #plt.imshow(depth) # <---- THIS LINE PREVENTS VIS.SHOW() FROM SHOWING
    plt.show()

if METHOD == "nogl":
    do_snapshot()
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
else:
    vis.setBackgroundColor(1,0,0)
    vis.show()
    while vis.shown():
        time.sleep(0.1)
        vis.show(False)

    vis.threadCall( do_snapshot)

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

    vis.kill()


print("QUITTING")
quit()
