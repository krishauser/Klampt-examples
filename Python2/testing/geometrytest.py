from klampt.model.create import *
from klampt import vis
from klampt.math import vectorops,so3,se3
from klampt.model.trajectory import Trajectory
from klampt.model import config

a = box(0.1,0.5,1.0,center=(0,0,0),type='GeometricPrimitive')
b = sphere(0.4,center=(0,0,0),type='GeometricPrimitive')
geomtypes = ['GeometricPrimitive','TriangleMesh','PointCloud','VolumeGrid']
tparams = {'PointCloud':0.05,'VolumeGrid':0.04}
atypes = dict((t,a.convert(t,tparams.get(t,0))) for t in geomtypes)
btypes = dict((t,b.convert(t,tparams.get(t,0))) for t in geomtypes)

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

def convert(geom,type,label):
    global a,b,Ta,Tb,atypes,btype
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

vis.addAction(lambda:convert(b,'GeometricPrimitive','B'),"B to GeometricPrimitive")
vis.addAction(lambda:convert(b,'TriangleMesh','B'),"B to TriangleMesh")
vis.addAction(lambda:convert(b,'PointCloud','B'),"B to PointCloud")
vis.addAction(lambda:convert(b,'VolumeGrid','B'),"B to VolumeGrid")

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
    if mode == 'collision':
        if a.collides(b):
            vis.setColor('B',1,1,0,0.5)
        else:
            vis.setColor('B',0,1,0,0.5)
    elif mode == 'near':
        if a.withinDistance(b,0.05):
            vis.setColor('B',1,1,0,0.5)
        else:
            vis.setColor('B',0,1,0,0.5)
    elif mode == 'distance':
        settings = DistanceQuerySettings()
        try:
            res = a.distance_ext(b,settings)
        except Exception as e:
            print "Exception encountered:",e
            continue
        if res.d < 0:
            vis.setColor('B',1,1,0,0.5)
        else:
            vis.setColor('B',0,1,0,0.5)
        if res.hasClosestPoints:
            if abs(abs(res.d) - vectorops.distance(res.cp1,res.cp2)) > 1e-7:
                print "ERROR IN DISTANCE?",res.d,vectorops.distance(res.cp1,res.cp2)
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
            print "Exception encountered:",e
            continue
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
                print "ERROR IN DEPTH?",res.depths[i],vectorops.distance(p1,p2)
        vis.unlock()
        
vis.kill()
