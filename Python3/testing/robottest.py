from klampt import *
from klampt import vis
import sys

if len(sys.argv)<=1:
    fn = "../../data/baxter_apc.xml"
else:
    fn = sys.argv[1]


w = WorldModel()
w.readFile(fn)
w.makeRobot("reduced")
dofmap = w.robot(1).reduce(w.robot(0))

vis.add("robot",w.robot(0))
vis.add("reduced",w.robot(1),color=(1,0,0))
vis.edit("reduced")
vis.loop()
