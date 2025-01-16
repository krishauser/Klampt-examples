import pytest
import cv2
import matplotlib.pyplot as plt
from klampt import vis
from klampt import WorldModel,TerrainModel
import numpy as np
import time

#this doesn't really matter
SHOW_OPENCV_FIRST = True

@pytest.fixture(scope = 'module')
def cap():
    cap = cv2.VideoCapture(0)
    assert cap.isOpened(),"Can't open camera... need to perform this test on a computer with a camera"
    ret, frame = cap.read()
    assert ret,"Can't receive frame"
    yield cap
    cap.release()

@pytest.fixture(scope = 'module')
def world():
    w = WorldModel()
    res = w.readFile("../../../data/athlete_plane.xml")
    assert res,"Failed to read world file"
    return w

def texturize_terrain(terrain : TerrainModel, frame : np.ndarray):
    bmin,bmax = terrain.geometry().getBB()
    app = terrain.appearance()
    app.setTexture2D('bgr8',frame)
    texgen = np.zeros((2,4))
    texgen[0,0] = 1.0/(bmax[0]-bmin[0])
    texgen[0,3] = -bmin[0]/(bmax[0]-bmin[0])
    texgen[1,1] = 1.0/(bmax[1]-bmin[1])
    texgen[1,3] = -bmin[1]/(bmax[1]-bmin[1])
    app.setTexgen(texgen)
    app.setColor(1,1,1,1)

def test_opencv_window_singlethread(cap,world):
    # Display the resulting frame
    if SHOW_OPENCV_FIRST:
        ret, frame = cap.read()
        assert ret,"Can't receive frame"
        cv2.imshow('frame', frame)

    vis.add("world",world)
    count = 50
    def capture(w=world,data=[count]):
        # Read from camera
        ret, frame = cap.read()
        assert ret
        
        # Display the resulting frame
        cv2.imshow('frame', frame)
        # Set as Klampt texture
        texturize_terrain(w.terrain(0),frame)
        data[0] -= 1
        if data[0] < 0:
            vis.show(False)
        
    vis.loop(callback=capture)
    cv2.destroyWindow("frame") 
    vis.kill()

def test_opencv_mpl_window_singlethread(cap,world):
    """Tests both opencv and matplotlib at the same time"""
    # Display the resulting frame
    ret, frame = cap.read()
    assert ret,"Can't receive frame"
    if SHOW_OPENCV_FIRST:
        cv2.imshow('frame', frame)
    plt.ion()
    plt.imshow(frame,interpolation='none')
    plt.show()
    
    count = 50
    vis.add("world",world)
    def capture(w=world,data=[count]):
        # Read from camera
        ret, frame = cap.read()
        assert ret            
        
        # Display the resulting frame
        cv2.imshow('frame', frame)
        # Set as Klampt texture
        texturize_terrain(w.terrain(0),frame)
        # Show in matplotlib
        plt.imshow(frame[:,:,::-1],interpolation='none')
        data[0] -= 1
        if data[0] < 0:
            vis.show(False)

    vis.loop(callback=capture)
    plt.close('all')
    cv2.destroyWindow('frame')
    vis.kill()

def _test_opencv_capture_vis_multithread(cap,world):
    """Tests capturing from an opencv camera in the vis loop"""
    # Display the resulting frame
    if SHOW_OPENCV_FIRST:
        ret, frame = cap.read()
        assert ret,"Can't receive frame"
        # cv2.imshow('frame', frame)

    vis.add("world",world)
    vis.show()
    count = 50
    while vis.shown() and count > 0:
        ret, frame = cap.read()
        assert ret
        vis.lock()
        texturize_terrain(world.terrain(0),frame)
        vis.unlock()
        time.sleep(0.02)
        count -= 1
    vis.show(False)
    vis.kill()

def _test_opencv_window_multithread(cap,world):
    """Tests capturing from an opencv camera and showing the window in the multithreaded
    vis loop -- doesn't work unless you create opencv windows within the vis thread."""
    vis.add("world",world)
    vis.show()
    count = 50
    while vis.shown() and count > 0:
        ret, frame = cap.read()
        if ret:
            vis.lock()
            texturize_terrain(world.terrain(0),frame)
            vis.unlock()
            #this is how you can create opencv windows in the same thread
            vis.threadCall(lambda : cv2.imshow('frame', frame))  
        time.sleep(0.02)
        count -= 1
    vis.show(False)
    vis.threadCall(lambda : cv2.destroyWindow('frame'))  
    vis.kill()

def _test_opencv_mpl_window_multithread(cap, world):
    """This doesn't really work.  Matplotlib doesn't like being run in a separate thread."""
    ret, frame = cap.read()
    assert ret,"Can't receive frame"
    def showfirstframe(frame=frame):
        plt.ion()
        plt.imshow(frame,interpolation='none')
        plt.show()

    vis.add("world",world)
    vis.show()
    vis.threadCall(showfirstframe)

    count = 50
    while vis.shown() and count > 0:
        ret, frame = cap.read()
        if ret:
            vis.lock()
            texturize_terrain(world.terrain(0),frame)
            vis.unlock()
            #this is how you can create MPL windows in the same thread
            vis.threadCall(lambda : plt.imshow(frame[:,:,::-1],interpolation='none'))
            #this is how you can create opencv windows in the same thread
            vis.threadCall(lambda : cv2.imshow('frame', frame))  
        time.sleep(0.02)
        count -= 1
    vis.show(False)
    vis.threadCall(lambda : cv2.destroyWindow('frame'))  
    vis.threadCall(lambda : plt.close('all'))

    
if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    assert cap.isOpened(),"Can't open camera... need to perform this test on a computer with a camera"
    ret, frame = cap.read()
    assert ret,"Can't receive frame"

    w = WorldModel()
    res = w.readFile("../../../data/athlete_plane.xml")
    assert res,"Failed to read world file"
    
    print("THIS NEXT TEST IS EXPECTED TO FAIL")
    _test_opencv_mpl_window_multithread(cap,w)

    cap.release()
