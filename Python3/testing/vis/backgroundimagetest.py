from klampt import *
import time

def make_world():
    w = WorldModel()
    w.readFile("../../../data/tx90cupscupboard.xml")
    #w.readFile("../../../data/robots/tx90ball.rob")
    r = w.robot(0)
    q0 = r.getConfig()
    r.randomizeConfig()
    qrand = r.getConfig()
    r.setConfig(q0)
    r.randomizeConfig()
    qrand2 = r.getConfig()
    r.setConfig(q0)
    return w

def test_background_image():
    import numpy as np
    from PIL import Image
    im = Image.open("../../../data/simulation_test_worlds/hauser.bmp")
    w = make_world()
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

if __name__ == '__main__':
    test_background_image()

    