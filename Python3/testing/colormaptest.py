from klampt.model.create import *
from klampt import vis
from klampt.robotsim import Geometry3D,WorldModel
from klampt.math import se3,so3,vectorops
from klampt.vis import colorize
import sys
import time

if len(sys.argv) > 1:
    a = Geometry3D()
    if not a.loadFile(sys.argv[1]):
        print("Error loading",sys.argv[1])
        exit()
else:
    a = sphere(0.4,center=(0,0,0),type='TriangleMesh')
    a = Geometry3D(a)

w = WorldModel()
w.makeRigidObject("a")
w.rigidObject(0).geometry().set(a)

#a_pc = a.convert("PointCloud",0.05)
bb = a.getBBTight()
a_pc = a.convert("PointCloud")
a_pc.setCurrentTransform(so3.identity(),[1.25*(bb[1][0]-bb[0][0]),0,0])
value = None
cmap = None
feature = None
lighting = None

vis.add("A",a)
vis.add("B",a_pc)

def convert(value_new,cmap_new,feature_new=None,lighting_new=None):
    global value,cmap,feature,lighting,w
    if value_new is None or value_new == value:
        if cmap_new is None or cmap == cmap_new:
            if feature_new is None or feature == feature_new:
                if lighting_new is None or feature == lighting_new:
                    return
    if cmap_new is not None:
        cmap = cmap_new
    if value_new is not None:
        value = value_new
    if feature_new is not None:
        feature = feature_new
    if lighting_new is not None:
        lighting = lighting_new
        if lighting_new == 'none':
            lighting = None
    if value is None:
        value = 'z'
    if cmap is None:
        cmap = 'viridis'
    a_app = colorize.colorize(w.rigidObject(0),value,cmap,feature,lighting=lighting)
    #a_app.drawGL(a)
    #if not value.startswith('n'):
    colorize.colorize(a_pc,value,cmap,lighting=lighting)
    vis.remove("A")
    vis.add("A",a_app)
    vis.remove("B")
    vis.add("B",a_pc)
    vis.dirty("B")
    #print("PC First color %08x"%(int(a_pc.getPointCloud().getProperty(0,'rgba')),))

convert('z',None,'faces')

vis.addAction(lambda:convert(None,'viridis'),"viridis colormap")
vis.addAction(lambda:convert(None,'plasma'),"plasma colormap")
vis.addAction(lambda:convert(None,'random'),"random colormap")
vis.addAction(lambda:convert('position',None),"position value")
vis.addAction(lambda:convert('x',None),"x value")
vis.addAction(lambda:convert('y',None),"y value")
vis.addAction(lambda:convert('z',None),"z value")
vis.addAction(lambda:convert('normal',None),"normal value")
vis.addAction(lambda:convert('nx',None),"nx value")
vis.addAction(lambda:convert('ny',None),"ny value")
vis.addAction(lambda:convert('nz',None),"nz value")
vis.addAction(lambda:convert(None,None,'faces'),"colorize faces")
vis.addAction(lambda:convert(None,None,'vertices'),"colorize vertices")
vis.addAction(lambda:convert(None,None,None,[0,0,-1]),"lighting on")
vis.addAction(lambda:convert(None,None,None,'none'),"lighting off")


vis.show()
while vis.shown():
    time.sleep(0.001)
vis.kill()
