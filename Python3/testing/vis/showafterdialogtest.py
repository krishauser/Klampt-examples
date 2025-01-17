from klampt import *
from klampt import vis
from klampt.io import resource
import time

print ("Tests vis show() after vis.dialog()")

world = WorldModel()
world.readFile('../../../data/objects/block.obj')
resource.edit("object transform",world.rigidObject(0).getTransform(),world=world)

def launchdialog():
    resource.edit("object transform launched from window",world.rigidObject(0).getTransform(),world=world)

def launchwindow():
    origwindow = vis.getWindow()
    vis.createWindow("Pop up window")
    vis.add("world2",world)
    vis.show()
    vis.setWindow(origwindow)

vis.add("world",world)
vis.addAction(launchdialog,"Launch a dialog","d")
vis.addAction(launchwindow,"Launch a window","w")
if not vis.multithreaded():
    print ("Now running loop()")
    vis.loop()
else:
    print ("Now running show() (only works on multithreaded systems, not mac)")
    vis.show()
    while vis.shown():
        time.sleep(0.1)
    vis.kill()