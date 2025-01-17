from klampt.model.create import *
from klampt import vis
from klampt.robotsim import Geometry3D,WorldModel
from klampt.math import se3,so3,vectorops
from klampt.vis import colorize
import sys
import time
import math

if len(sys.argv) > 1:
    a = Geometry3D()
    if not a.loadFile(sys.argv[1]):
        print("Error loading",sys.argv[1])
        exit()
else:
    a = sphere(0.4,center=(0,0,0),resolution=0.02,type='TriangleMesh')
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
    a_app = colorize.colorize(a,value,cmap,feature,lighting=lighting)
    colorize.colorize(a_pc,value,cmap,lighting=lighting)
    #vis.remove("A")
    vis.add("A",a,appearance=a_app)   #a is a TriangleMesh geometry and colorize returns an Appearance
    #vis.remove("B")
    vis.add("B",a_pc)
    vis.dirty("B")
    
def convert_segmentation():
    #tests using an integer segmentation to colorize the object 
    global cmap,lighting
    points = a.getTriangleMesh().vertices
    tris = a.getTriangleMesh().indices
    segments = []  #split by face z
    for t in tris:
        lower = min(points[t[0]][2],points[t[1]][2],points[t[2]][2])
        segments.append(int(math.floor(lower*7)))
    a_app = colorize.colorize(a,segments,cmap,feature='faces')
    vis.add("A",a,appearance=a_app)   #a is a TriangleMesh geometry and colorize returns an Appearance

    pcdata = a_pc.getPointCloud()
    segindex = pcdata.propertyIndex('segment')
    if segindex < 0:
        segindex = pcdata.addProperty('segment')
        assert pcdata.propertyIndex('segment') >= 0
    points = pcdata.points
    segments = [int(math.floor(p[2]*7)) for p in points]  #split by z
    pcdata.properties[:,segindex] = segments
    assert a_pc.getPointCloud().propertyIndex('segment') >= 0  #did this write to the pc data?
    colorize.colorize(a_pc,'segment',cmap)
    vis.add("B",a_pc)
    vis.dirty("B")

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
vis.addAction(convert_segmentation,"segment value")
vis.addAction(lambda:convert(None,None,'faces'),"colorize faces")
vis.addAction(lambda:convert(None,None,'vertices'),"colorize vertices")
vis.addAction(lambda:convert(None,None,None,[0,0,-1]),"lighting on")
vis.addAction(lambda:convert(None,None,None,'none'),"lighting off")


vis.show()
while vis.shown():
    time.sleep(0.001)
vis.kill()
