from klampt.model.create import *
from klampt import vis
from klampt.math import vectorops,so3,se3
from klampt.model.trajectory import Trajectory
from klampt.model import config
from klampt.robotsim import Geometry3D,WorldModel,DistanceQuerySettings
from collections import deque
import sys
import time
import math

if len(sys.argv) > 1:
    a = Geometry3D()
    if not a.loadFile(sys.argv[1]):
        print("Error loading",sys.argv[1])
        exit()
else:
    a = box(0.1,0.5,1.0,center=(0,0,0),type='GeometricPrimitive')
if len(sys.argv) > 2:
    b = Geometry3D()
    if not b.loadFile(sys.argv[2]):
        print("Error loading",sys.argv[2])
        exit()
else:
    b = sphere(0.4,center=(0,0,0),type='GeometricPrimitive')
    #seg = GeometricPrimitive()
    #seg.setSegment([-0.25,0,0],[0.25,0,0])
    #b = Geometry3D(seg)


geomtypes = ['GeometricPrimitive','TriangleMesh','PointCloud','VolumeGrid','ConvexHull']
tparams = {'PointCloud':0.02,'VolumeGrid':0.04}
atypes = dict()
btypes = dict()
for t in geomtypes:
    atypes[t] = a
    btypes[t] = b
    try:
        if a.type() != t:
            at = a.convert(t,tparams.get(t,0))
            atypes[t] = at
    except Exception as e:
        pass
    try:
        if b.type() != t:
            bt = b.convert(t,tparams.get(t,0))
            btypes[t] = bt
    except Exception as e:
        pass

for (t,g) in atypes.items():
    m = Mass()
    m.estimate(g,1.0,1.0)
    print(t)
    print("  COM surface-only %.3f %.3f %.3f"%tuple(m.getCom()))
    H = m.getInertia()
    print("  Inertia surface-only %.3f %.3f %.3f"%(H[0],H[4],H[8]))
    m.estimate(g,1.0,0.0)
    H = m.getInertia()
    print("  COM volume-only %.3f %.3f %.3f"%tuple(m.getCom()))
    print("  Inertia volume-only %.3f %.3f %.3f"%(H[0],H[4],H[8]))

a = atypes['GeometricPrimitive']
b = btypes['GeometricPrimitive']
vis.add("A",a)
vis.add("B",b)
vis.setColor("A",1,0,0,0.5)
vis.setColor("B",0,1,0,0.5)
Ta = se3.identity()
Tb = [so3.identity(),[1,0,0]]
vis.add("Ta",Ta)
vis.add("Tb",Tb)
vis.edit("Ta")
vis.edit("Tb")

ray = ([-3,0,0],[1,0,0])
vis.add("ray",Trajectory([0,1],[ray[0],vectorops.madd(ray[0],ray[1],20)]),color=[1,0.5,0,1])
vis.add("hitpt",[0,0,0],color=[1,0,1,1])

def convert(geom,type,label):
    global a,b,atypes,btypes,Ta,Tb
    if label=='A':
        vis.add(label,atypes[type])
        vis.setColor(label,1,0,0,0.5)
        a = atypes[type]
        a.setCurrentTransform(*Ta)
    else:
        vis.add(label,btypes[type])
        vis.setColor(label,0,1,0,0.5)
        b = btypes[type]
        b.setCurrentTransform(*Tb)

vis.addAction(lambda:convert(a,'GeometricPrimitive','A'),"A to GeometricPrimitive")
vis.addAction(lambda:convert(a,'TriangleMesh','A'),"A to TriangleMesh")
vis.addAction(lambda:convert(a,'PointCloud','A'),"A to PointCloud")
vis.addAction(lambda:convert(a,'VolumeGrid','A'),"A to VolumeGrid")
vis.addAction(lambda:convert(a,'ConvexHull','A'),"A to ConvexHull")

vis.addAction(lambda:convert(b,'GeometricPrimitive','B'),"B to GeometricPrimitive")
vis.addAction(lambda:convert(b,'TriangleMesh','B'),"B to TriangleMesh")
vis.addAction(lambda:convert(b,'PointCloud','B'),"B to PointCloud")
vis.addAction(lambda:convert(b,'VolumeGrid','B'),"B to VolumeGrid")
vis.addAction(lambda:convert(b,'ConvexHull','B'),"B to ConvexHull")


global mode
global drawExtra
mode = 'collision'
drawExtra = set()
def setMode(m):
    global mode,drawExtra
    mode = m
    vis.lock()
    for s in drawExtra:
        vis.remove(s)
    drawExtra = set()
    vis.unlock()

vis.addAction(lambda:setMode('collision'),'Collision mode','c')
vis.addAction(lambda:setMode('near'),'Near mode','n')
vis.addAction(lambda:setMode('distance'),'Distance mode','d')
vis.addAction(lambda:setMode('contacts'),'Contacts mode','k')



def remesh(geom,label):
    global a,b,Ta,Tb
    if label=='A':
        a = a.convert(a.type(),0.06)
        vis.add(label,a)
        vis.setColor(label,1,0,0,0.5)
        a.setCurrentTransform(*Ta)
    elif label=='B':
        b = b.convert(b.type(),0.06)
        vis.add(label,b)
        vis.setColor(label,0,1,0,0.5)
        b.setCurrentTransform(*Tb)

vis.addAction(lambda:remesh(a,'A'),"A remesh")
vis.addAction(lambda:remesh(b,'B'),"B remesh")


def doSlice():
    global a,b
    T = se3.identity()
    ares = a.slice(T[0],T[1],0.05)
    bres = b.slice(T[0],T[1],0.05)
    ares.setCurrentTransform(T[0],T[1])
    bres.setCurrentTransform(T[0],T[1])
    if ares.type() != '':
        vis.add("A slice",ares,color=(1,0.5,0,1))
    else:
        try:
            vis.remove("A slice")
        except Exception:
            pass
    if bres.type() != '':
        vis.add("B slice",bres,color=(0,1,0.5,1))
    else:
        try:
            vis.remove("B slice")
        except Exception:
            pass

def doROI():
    global a,b
    bmin = [-1,0,0]
    bmax = [1,.5,0.1]
    ares = a.roi('intersect',bmin,bmax)
    bres = b.roi('intersect',bmin,bmax)
    print(ares.getCurrentTransform())
    print(bres.getCurrentTransform())
    if ares.type() != '':
        vis.add("A roi",ares,color=(1,0.5,0,1))
    else:
        try:
            vis.remove("A roi")
        except Exception:
            pass
    if bres.type() != '':
        vis.add("B roi",bres,color=(0,1,0.5,1))
    else:
        try:
            vis.remove("B roi")
        except Exception:
            pass

vis.addAction(doSlice,'Make slices','s')
vis.addAction(doROI,'Make ROI','r')


#Testing different types of geometric primitives
"""
prim1 = GeometricPrimitive()
prim1.setTriangle([0,0,1],[0.5,0,1],[0,0.5,1])
vis.add("prim1",prim1,color=[0.5,0.5,1])
prim2 = GeometricPrimitive()
prim2.setPolygon([0,0,1.2]+[0.5,0,1.2]+[0.5,0.5,1.2]+[0,0.5,1.2])
vis.add("prim2",prim2,color=[0.5,1,0.5])
prim3 = GeometricPrimitive()
prim3.setBox([0,0,1.5],so3.rotation([0,1,0],math.radians(10)),[0.5,0.5,0.5])
vis.add("prim3",prim3,color=[1,0.5,0.5])
"""

#Testing polygon - ray intersection
"""
polygon = GeometricPrimitive()
polygon.setPolygon([0,0,0,  1,0,0,  0.5,0.5,0, 1,1,0,  0,1,0])
c = Geometry3D(polygon)
c.setCurrentTransform(so3.identity(),[0,0,-0.5])
vis.add("polygon",c,color=[0.2,0,0.6,1])
for i in range(10):
    x = i*0.2 - 0.2
    y = 0.5
    z = 1
    s,d = [x,y,z],vectorops.unit([0,-0.2,-1])
    elem,pt = c.rayCast_ext(s,d)
    vis.add("ray "+str(i),[s,vectorops.madd(s,d,2)],color=(1,1,0,1))
    if elem >= 0:
        vis.add("polygon hit "+str(i),pt,color=[0.4,0,1,1])
"""

#Testing groups
"""
c = Geometry3D()
c.setGroup()
c.setElement(0,a)
c.setElement(1,b)
#note: these transforms aren't respected
c.getElement(1).setCurrentTransform(so3.identity(),[0,1,0])
c2 = c.convert('TriangleMesh')
vis.add("c2",c2)

#debugging group geometry drawing
c = Geometry3D()
c.setGroup()
c.setElement(0,atypes['TriangleMesh'])
c.setElement(1,btypes['TriangleMesh'])
w = WorldModel()
w.makeRigidObject('temp')
w.rigidObject(0).geometry().set(c)
app = w.rigidObject(0).appearance()
vis.add("c",w.rigidObject(0))
"""

#Testing remeshing
"""
a2 = a.convert('TriangleMesh',0.2)
vis.debug(a2)
vis.debug(a2.convert("PointCloud"))
"""

counter = 0
Ntimes = 30
last_cycle_times = deque()
last_query_times = deque()
t0 = time.time()
vis.show()
while vis.shown():
    qa = vis.getItemConfig("Ta")
    qb = vis.getItemConfig("Tb")
    config.setConfig(Ta,qa)
    config.setConfig(Tb,qb)
    vis.lock()
    a.setCurrentTransform(*Ta)
    b.setCurrentTransform(*Tb)
    vis.unlock()
    tq0 = time.time()
    if mode == 'collision':
        coll = a.collides(b)
        tq1 = time.time()
        if coll:
            vis.setColor('B',1,1,0,0.5)
        else:
            vis.setColor('B',0,1,0,0.5)
    elif mode == 'near':
        coll = a.withinDistance(b,0.05)
        tq1 = time.time()
        if coll:
            vis.setColor('B',1,1,0,0.5)
        else:
            vis.setColor('B',0,1,0,0.5)
    elif mode == 'distance':
        settings = DistanceQuerySettings()
        try:
            res = a.distance_ext(b,settings)
        except Exception as e:
            print("Exception encountered:",e)
            continue
        tq1 = time.time()
        if res.d < 0:
            vis.setColor('B',1,1,0,0.5)
        else:
            vis.setColor('B',0,1,0,0.5)
        if res.hasClosestPoints:
            if abs(abs(res.d) - vectorops.distance(res.cp1,res.cp2)) > 1e-7:
                print("ERROR IN DISTANCE?",res.d,vectorops.distance(res.cp1,res.cp2))
            vis.lock()
            vis.add("cpa",[v for v in res.cp1])
            vis.add("cpb",[v for v in res.cp2])
            vis.setColor("cpa",0,0,1)
            vis.setColor("cpb",0,0,1)
            vis.add("cpline",Trajectory([0,1],[[v for v in res.cp1],[v for v in res.cp2]]))
            vis.setColor("cpline",1,0.5,0)
            drawExtra.add("cpa")
            drawExtra.add("cpb")
            drawExtra.add("cpline")
            if res.hasGradients:
                l = 0.1
                vis.add("cpagrad",vectorops.madd(res.cp1,res.grad1,l))
                vis.add("cpbgrad",vectorops.madd(res.cp2,res.grad2,l))
                vis.setColor('cpagrad',1,1,0)
                vis.setColor('cpbgrad',1,1,0)
                drawExtra.add("cpagrad")
                drawExtra.add("cpbgrad")
            vis.unlock()
    elif mode == 'contacts':
        try:
            res = a.contacts(b,0.05,0.01)
        except Exception as e:
            print("Exception encountered:",e)
            continue
        tq1 = time.time()
        vis.lock()
        for s in drawExtra:
            vis.remove(s)
        drawExtra = set()
        for i,(p1,p2) in enumerate(zip(res.points1,res.points2)):
            vis.add("cpa"+str(i),[v for v in p1],hide_label=True,color=[1,1,0,1],size=5)
            vis.add("cpb"+str(i),[v for v in p2],hide_label=True,color=[1,0.5,0,1],size=5)
            l = 0.1
            vis.add("n"+str(i),Trajectory([0,1],[[v for v in p1],vectorops.madd(p1,res.normals[i],l)]),hide_label=True,color=[1,0,0,1])
            drawExtra.add("cpa"+str(i))
            drawExtra.add("cpb"+str(i))
            drawExtra.add("n"+str(i))
            if abs(abs(res.depths[i]) - vectorops.distance(p1,p2)) > 1e-7:
                print("ERROR IN DEPTH?",res.depths[i],vectorops.distance(p1,p2))
        vis.unlock()
    elem,pt = a.rayCast_ext(ray[0],ray[1])
    if elem >= 0:
        vis.add("hitpt",pt)
        vis.setColor("hitpt",0,0,0)
    else:
        vis.setColor("hitpt",1,0,1)

    time.sleep(0.001)
    t1 = time.time()

    last_cycle_times.append(t1-t0)
    while len(last_cycle_times) > Ntimes:
        last_cycle_times.popleft()
    last_query_times.append(tq1-tq0)
    while len(last_query_times) > Ntimes:
        last_query_times.popleft()
    counter += 1
    if counter % 30 == 0:
        vis.addText("counter","Cycle time %.3fms, query time %.3fms"%(1000*sum(last_cycle_times)/len(last_cycle_times),1000*sum(last_query_times)/len(last_query_times)),(10,10))
    t0 = t1
vis.kill()
