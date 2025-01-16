#!/usr/bin/python

import sys
import math
import klampt
from klampt import PointPoser,RobotPoser
from klampt.vis.glcommon import GLWidgetPlugin
from klampt import vis


class MyGLViewer(GLWidgetPlugin):
    def __init__(self,world):
        GLWidgetPlugin.__init__(self)
        self.world = world
        self.poseWidget = PointPoser()
        self.robotWidget = RobotPoser(world.robot(0))
        self.addWidget(self.poseWidget)
        self.addWidget(self.robotWidget)
        self.add_action(self.print_config,'Print config',' ')
        self.add_action(self.save_world,'Save world','s')

    def print_config(self):
        config = self.robotWidget.get()
        print("Config:",config)
        self.world.robot(0).setConfig(config)
    
    def save_world(self):
        fn = "widgets_test_world.xml"
        print("Saving file to",fn)
        self.world.saveFile(fn)



if __name__ == "__main__":
    print("==============================================================================")
    print("gl_vis_widgets.py: This example demonstrates how to manually add visualization")
    print("widgets to pose a robot using the GLWidgetPlugin interface")
    if len(sys.argv)<=1:
        print()
        print("USAGE: gl_vis_widgets.py [world_file]")
    print()
    print("Note: we recommend that beginners use the vis interface, as it is more user-")
    print("friendly and can often work with other visualization backends than PyQT/GLUT.")
    print("==============================================================================")
    if len(sys.argv)<=1:
        exit()
    world = klampt.WorldModel()
    for fn in sys.argv[1:]:
        res = world.readFile(fn)
        if not res:
            raise RuntimeError("Unable to load model "+fn)
    # viewer = MyGLViewer(world)
    # vis.run(viewer)
    # Equivalent code using vis interface    
    # import time
    # vis.init()
    # vis.add("world",world)
    # vis.add("point",(0.0,0.0,0.0))
    # pt_editor = vis.edit("point")
    # editor = vis.edit(("world",world.robot(0).getName()))
    # def print_config():
    #     print("Config:",editor.get())
    #     world.robot(0).setConfig(editor.get())
    # def save_world():
    #     fn = "widgets_test_world.xml"
    #     print("Saving file to",fn)
    #     world.saveFile(fn)
    # vis.addAction(print_config,'Print config',' ')
    # vis.addAction(save_world,'Save world','s')
    # vis.show()
    # while vis.shown():
    #     time.sleep(0.05)
    