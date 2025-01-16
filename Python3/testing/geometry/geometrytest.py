from klampt.model.create import *
from klampt import vis
from klampt import __version__ as klampt_version
from klampt.math import vectorops,so3,se3
from klampt.model.trajectory import Trajectory
from klampt.model import config
from klampt import Geometry3D,Appearance,WorldModel,DistanceQuerySettings
from collections import deque
import sys
import time

klampt_version = tuple(int(v) for v in klampt_version.split('.'))

if len(sys.argv) > 1:
    a = Geometry3D()
    if not a.loadFile(sys.argv[1]):
        print("Error loading",sys.argv[1])
        exit()
else:
    a = box(1.1,0.5,1.0,center=(0,0,0),type='GeometricPrimitive')
    #a = sphere(0.5,center=(0,0,0),type='GeometricPrimitive')
if len(sys.argv) > 2:
    b = Geometry3D()
    if not b.loadFile(sys.argv[2]):
        print("Error loading",sys.argv[2])
        exit()
else:
    b = sphere(0.4,center=(0,0,0),type='GeometricPrimitive')
    gp = b.getGeometricPrimitive() # type: GeometricPrimitive

    #seg = GeometricPrimitive()
    #seg.setSegment([-0.25,0,0],[0.25,0,0])
    #b = Geometry3D(seg)

abb = a.getBB()
size = max(vectorops.sub(abb[1],abb[0]))

if klampt_version >= (0,10,0):
    geomtypes = ['GeometricPrimitive','TriangleMesh','PointCloud','ImplicitSurface','OccupancyGrid','ConvexHull','Heightmap']
else:  #0.9 and below
    geomtypes = ['GeometricPrimitive','TriangleMesh','PointCloud','VolumeGrid']
tparams = {'PointCloud':size*0.02,'VolumeGrid':size*0.04,'ImplicitSurface':size*0.04,'OccupancyGrid':size*0.04,'Heightmap':size*0.01}
tparamsb = {'PointCloud':size*0.01,'VolumeGrid':size*0.01,'ImplicitSurface':size*0.01,'OccupancyGrid':size*0.01,'Heightmap':size*0.005}
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
        print("Could not convert",a.type(),"to",t)
        atypes[t] = box(1.1,0.5,1.0,center=(0,0,0),type='GeometricPrimitive').convert(t,tparams.get(t,0))
    try:
        if b.type() != t:
            bt = b.convert(t,tparamsb.get(t,0))
            btypes[t] = bt
    except Exception as e:
        print("Could not convert",b.type(),"to",t)
        btypes[t] = sphere(0.4,center=(0,0,0),type='GeometricPrimitive').convert(t,tparamsb.get(t,0))


a = atypes[a.type()]
b = btypes[b.type()]
Ta = se3.identity()
Tb = [so3.identity(),[1,0,0]]
vis.add("A",a)
vis.add("B",b)
vis.setColor("A",1,0,0,0.5)
vis.setColor("B",0,1,0,0.5)
vis.add("Ta",Ta)
vis.add("Tb",Tb)
Ta_widget = vis.edit("Ta")
Tb_widget = vis.edit("Tb")

ray = ([-3,0,0],[1,0,0])

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
    refreshBB()

vis.addAction(lambda:convert(a,'GeometricPrimitive','A'),"A to GeometricPrimitive")
vis.addAction(lambda:convert(a,'TriangleMesh','A'),"A to TriangleMesh")
vis.addAction(lambda:convert(a,'PointCloud','A'),"A to PointCloud")
vis.addAction(lambda:convert(a,'ImplicitSurface','A'),"A to ImplicitSurface")
vis.addAction(lambda:convert(a,'OccupancyGrid','A'),"A to OccupancyGrid")
vis.addAction(lambda:convert(a,'ConvexHull','A'),"A to ConvexHull")
vis.addAction(lambda:convert(a,'Heightmap','A'),"A to Heightmap")

vis.addAction(lambda:convert(b,'GeometricPrimitive','B'),"B to GeometricPrimitive")
vis.addAction(lambda:convert(b,'TriangleMesh','B'),"B to TriangleMesh")
vis.addAction(lambda:convert(b,'PointCloud','B'),"B to PointCloud")
vis.addAction(lambda:convert(b,'ImplicitSurface','B'),"B to ImplicitSurface")
vis.addAction(lambda:convert(b,'OccupancyGrid','B'),"B to OccupancyGrid")
vis.addAction(lambda:convert(b,'ConvexHull','B'),"B to ConvexHull")
vis.addAction(lambda:convert(b,'Heightmap','B'),"B to Heightmap")


global mode
global drawExtra
global doRayCast
global showBBox
mode = 'collision'
drawExtra = set()
doRayCast = False
showBBox = False
def setMode(m):
    global mode,drawExtra
    mode = m
    vis.lock()
    for s in drawExtra:
        vis.remove(s)
    drawExtra = set()
    vis.unlock()
    vis.addText("mode","Mode: "+m,(10,0))

vis.addAction(lambda:setMode('collision'),'Collision mode','c')
vis.addAction(lambda:setMode('near'),'Near mode','n')
vis.addAction(lambda:setMode('distance'),'Distance mode','d')
vis.addAction(lambda:setMode('distance_point'),'Distance to point mode','p')
vis.addAction(lambda:setMode('contains_point'),'Point containment mode','i')
vis.addAction(lambda:setMode('contacts'),'Contacts mode','k')

def toggleRayCast():
    global doRayCast, ray
    doRayCast = not doRayCast
    if doRayCast:
        vis.add("ray",Trajectory([0,1],[ray[0],vectorops.madd(ray[0],ray[1],20)]),color=[1,0.5,0,1])
        vis.add("hitpt",[0,0,0],color=[1,0,1,1])
    else:
        vis.remove("ray")
        vis.remove("hitpt")

vis.addAction(toggleRayCast,'Toggle ray cast')

def refreshBB():
    global showBBox
    if showBBox:
        abb = a.getBB()
        prim = GeometricPrimitive()
        prim.setAABB(abb[0],abb[1])
        gab = Geometry3D(prim)
        vis.add("bbA",gab,color=[1,0,0,0.25])
        bbb = b.getBB()
        prim.setAABB(bbb[0],bbb[1])
        gbb = Geometry3D(prim)
        vis.add("bbB",gbb,color=[0,1,0,0.25])

def toggleShowBB():
    global showBBox
    showBBox = not showBBox
    if showBBox:
        refreshBB()
    else:
        vis.remove("bbA")
        vis.remove("bbB")
vis.addAction(toggleShowBB,'Toggle bounding box','b')

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
    slicepts_local = [(-1,-1),(1,-1),(1,1),(-1,1)]
    slicepts_world = [se3.apply(T,(pt[0],pt[1],0)) for pt in slicepts_local]
    vis.add("Slice",slicepts_world + [slicepts_world[0]],color=(0,0,1,1))

    try:
        ares = a.slice(T[0],T[1],0.05)
        ares.setCurrentTransform(T[0],T[1])
        if ares.type() != '':
            vis.add("A slice",ares,color=(1,0.5,0,1))
        else:
            try:
                vis.remove("A slice")
            except Exception:
                pass
    except Exception as e:
        print("Unable to compute slice for",a.type())
    try:
        bres = b.slice(T[0],T[1],0.05)
        bres.setCurrentTransform(T[0],T[1])
        if bres.type() != '':
            vis.add("B slice",bres,color=(0,1,0.5,1))
        else:
            try:
                vis.remove("B slice")
            except Exception:
                pass
    except Exception as e:
        print("Unable to compute slice for",b.type())

def doROI():
    global a,b
    bmin = [-1,0,0]
    bmax = [1,.5,0.1]
    print("Computing ROI with bounds",bmin,bmax)
    prim = GeometricPrimitive()
    prim.setAABB(bmin,bmax)
    gab = Geometry3D(prim)
    vis.add("ROI",gab,color=[0,0,1,0.25])

    try:
        ares = a.roi('intersect',bmin,bmax)
        if ares.type() != '':
            vis.add("A roi",ares,color=(1,0.5,0,1))
        else:
            try:
                vis.remove("A roi")
            except Exception:
                pass
    except Exception as e:
        print("Could not compute ROI for",a.type())
        pass

    try:
        bres = b.roi('intersect',bmin,bmax)
        if bres.type() != '':
            vis.add("B roi",bres,color=(0,1,0.5,1))
        else:
            try:
                vis.remove("B roi")
            except Exception:
                pass
    except Exception as e:
        print("Could not compute ROI for",b.type())
        pass

def doMerge():
    global a,b
    if klampt_version >= (0,10,0):
        atemp = a.copy()
    else:
        atemp = a.clone()
    try:
        atemp.merge(b)
        vis.add("Merged",atemp,color=(1,0.5,0,1))
    except Exception as e:
        print("Could not merge",b.type(),"into",a.type())
        pass

vis.addAction(doSlice,'Make slices','s')
vis.addAction(doROI,'Make ROI','r')
vis.addAction(doMerge,'Make merge','m')



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
    if Ta_widget.hasFocus() or Tb_widget.hasFocus():
        refreshBB()
    tq0 = time.time()
    if mode == 'collision':
        if klampt_version >= (0,10,0):
            elems1,elems2 = a.collides_ext(b,100)
            coll = len(elems1) > 0
            assert len(elems1) == len(elems2)
        else:  #earlier versions of Klampt
            coll = a.collides(b)
        tq1 = time.time()
        vis.lock()
        for s in drawExtra:
            vis.remove(s)
        drawExtra = set()
        if coll:
            vis.setColor('B',1,1,0,0.5)
            if klampt_version >= (0,10,0):
                for (i,j) in zip(elems1,elems2):
                    if a.numElements() > 1 and i >= 0:
                        ei = a.getElement(i)
                        ei.setCurrentTransform(*Ta)
                        if ei.type() == 'GeometricPrimitive' and ei.getGeometricPrimitive().type == 'Point':
                            g = ei.getGeometricPrimitive()
                            pt = se3.apply(Ta,[v for v in g.properties])
                            vis.add("witness_a_"+str(i),pt,hide_label=True,size=10.0,color=[1,0.5,0,1])
                        else:
                            vis.add("witness_a_"+str(i),ei,hide_label=True,color=[1,0.5,0,1])
                        drawExtra.add("witness_a_"+str(i))
                    if b.numElements() > 1 and j >= 0:
                        ej = b.getElement(j)
                        ej.setCurrentTransform(*Tb)
                        if ej.type() == 'GeometricPrimitive' and ej.getGeometricPrimitive().type == 'Point':
                            g = ej.getGeometricPrimitive()
                            pt = se3.apply(Tb,[v for v in g.properties])
                            vis.add("witness_b_"+str(j),pt,hide_label=True,size=10.0,color=[0.5,1,0,1])
                        else:
                            vis.add("witness_b_"+str(j),ej,hide_label=True,color=[0.5,1,0,1])
                        drawExtra.add("witness_b_"+str(j))
        else:
            vis.setColor('B',0,1,0,0.5)
        vis.unlock()
    elif mode == 'near':
        if klampt_version >= (0,10,0):
            elems1,elems2 = a.withinDistance_ext(b,0.06,100)
            coll = len(elems1) > 0
            assert len(elems1) == len(elems2)
        else:  #earlier versions of Klampt
            coll = a.withinDistance(b,0.06)
        tq1 = time.time()
        vis.lock()
        for s in drawExtra:
            vis.remove(s)
        drawExtra = set()
        if coll:
            vis.setColor('B',1,1,0,0.5)
            if klampt_version >= (0,10,0):
                for (i,j) in zip(elems1,elems2):
                    if a.numElements() > 1:
                        ei = a.getElement(i)
                        ei.setCurrentTransform(*Ta)
                        if ei.type() == 'GeometricPrimitive' and ei.getGeometricPrimitive().type == 'Point':
                            g = ei.getGeometricPrimitive()
                            pt = se3.apply(Ta,[v for v in g.properties])
                            vis.add("witness_a_"+str(i),pt,hide_label=True,size=10.0,color=[1,0.5,0,1])
                        else:
                            vis.add("witness_a_"+str(i),ei,hide_label=True,color=[1,0.5,0,1])
                        drawExtra.add("witness_a_"+str(i))
                    if b.numElements() > 1:
                        ej = b.getElement(j)
                        ej.setCurrentTransform(*Tb)
                        if ej.type() == 'GeometricPrimitive' and ej.getGeometricPrimitive().type == 'Point':
                            g = ej.getGeometricPrimitive()
                            pt = se3.apply(Tb,[v for v in g.properties])
                            vis.add("witness_b_"+str(j),pt,hide_label=True,size=10.0,color=[0.5,1,0,1])
                        else:
                            vis.add("witness_b_"+str(j),ej,hide_label=True,color=[0.5,1,0,1])
                        drawExtra.add("witness_b_"+str(j))
        else:
            vis.setColor('B',0,1,0,0.5)
        vis.unlock()
    elif mode.startswith('distance'):
        settings = DistanceQuerySettings()
        if mode == 'distance':
            try:
                res = a.distance_ext(b,settings)
            except Exception as e:
                print("Exception encountered during Geometry3D.distance_ext():",e)
                continue
        else:
            assert mode == 'distance_point'
            try:
                res = a.distance_point_ext(Tb[1],settings)
            except Exception as e:
                print("Exception encountered during Geometry3D.distance_point_ext():",e)
                continue
        tq1 = time.time()
        if res.d < 0:
            vis.setColor('B',1,1,0,0.5)
        else:
            vis.setColor('B',0,1,0,0.5)
        if res.hasClosestPoints:
            if abs(abs(res.d) - vectorops.distance(res.cp1,res.cp2)) > 1e-5:
                print("ERROR IN DISTANCE? reported {0:.4f}, cp distance {0:.4f}".format(res.d,vectorops.distance(res.cp1,res.cp2)))
            vis.lock()
            vis.add("cpa",[v for v in res.cp1])
            vis.add("cpb",[v for v in res.cp2])
            vis.setColor("cpa",0,0,1)
            vis.setColor("cpb",0,0,1)
            vis.add("cpline",Trajectory([0,1],[[v for v in res.cp1],[v for v in res.cp2]]),hide_label=True)
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
    elif mode == 'contains_point':
        #test a grid of points
        for i in range(-10,10):
            for j in range(-10,10):
                p = [i*0.02,j*0.02,0]
                if a.contains_point(p):
                    vis.add("p"+str(i)+","+str(j),p,hide_label=True)
                    vis.setColor("p"+str(i)+","+str(j),0,1,0)
                else:
                    vis.add("p"+str(i)+","+str(j),p,hide_label=True)
                    vis.setColor("p"+str(i)+","+str(j),1,1,0)
        tq1 = time.time()
    elif mode == 'contacts':
        try:
            res = a.contacts(b,0.05,0.01)
        except Exception as e:
            print("Exception encountered during Geometry3D.contacts():",e)
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
            if abs(abs(res.depths[i]) - vectorops.distance(p1,p2)) > 1e-5:
                print("ERROR IN DEPTH? reported {0:.4f}, cp distance {0:.4f}".format(res.depths[i],vectorops.distance(p1,p2)))
        vis.unlock()

    if doRayCast:
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
