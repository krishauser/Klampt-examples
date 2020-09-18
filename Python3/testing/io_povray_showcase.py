from klampt import WorldModel,Geometry3D,GeometricPrimitive,VolumeGrid,PointCloud,Appearance
from klampt.vis import GLProgram,camera,gldraw
from klampt.io import povray,povray_animation
import klampt.math.vectorops as op
import klampt.math.se3 as se3
import klampt.math.so3 as so3
import math,os,random,vapory as vp

world=WorldModel()

#example Floor
floor=world.makeRigidObject("Floor")
prim=GeometricPrimitive()
prim.setAABB((-.1,-.1,-0.1),(.2,1.6,0.))
floor.geometry().setGeometricPrimitive(prim)
floor.appearance().setColor(.5,.5,.5)

#example Point
point=world.makeRigidObject("Point")
prim=GeometricPrimitive()
prim.setPoint((0.05,0.0,0.05))
point.geometry().setGeometricPrimitive(prim)
point.appearance().setColor(random.uniform(0.,1.),random.uniform(0.,1.),random.uniform(0.,1.),1.)

#example Sphere
point=world.makeRigidObject("Sphere")
prim=GeometricPrimitive()
prim.setSphere((0.05,0.1,0.05),0.02)
point.geometry().setGeometricPrimitive(prim)
point.appearance().setColor(random.uniform(0.,1.),random.uniform(0.,1.),random.uniform(0.,1.),1.)

#example Segment
point=world.makeRigidObject("Segment")
prim=GeometricPrimitive()
prim.setSegment((0.05,0.2,0.0),(0.05,0.2,0.1))
point.geometry().setGeometricPrimitive(prim)
point.appearance().setColor(random.uniform(0.,1.),random.uniform(0.,1.),random.uniform(0.,1.),1.)

#example AABB
aabb=world.makeRigidObject("AABB")
prim=GeometricPrimitive()
prim.setAABB((0.,0.3,0.),(0.1,0.4,0.1))
aabb.geometry().setGeometricPrimitive(prim)
aabb.appearance().setColor(random.uniform(0.,1.),random.uniform(0.,1.),random.uniform(0.,1.),0.5)

#example TriangleMesh
ladder=world.makeTerrain("TriangleMesh")
mesh=Geometry3D()
mesh.loadFile("../../data/terrains/drc_ladder.off")
mesh.transform([.033,0,0,0,.033,0,0,0,.033],[0.05,0.5,0])
ladder.geometry().set(mesh)
ladder.appearance().setColor(random.uniform(0.,1.),random.uniform(0.,1.),random.uniform(0.,1.),1.)

#example Group
ladders=world.makeTerrain("Group")
ladders_geom=Geometry3D()
ladders_geom.setGroup()
for d in range(4):
    mesh=Geometry3D()
    mesh.loadFile("../../data/terrains/drc_ladder.off")
    mesh.transform(so3.from_axis_angle(([0,0,1],math.pi/2*d)),[0,0,0])
    mesh.transform([.033,0,0,0,.033,0,0,0,.033],[0.05,0.8,0])
    ladders_geom.setElement(d,mesh)
ladders.geometry().set(ladders_geom)
ladders.appearance().setColor(random.uniform(0.,1.),random.uniform(0.,1.),random.uniform(0.,1.),1.)

#example VolumeGrid
sdf=world.makeTerrain("SDF")
grid=VolumeGrid()
grid.setBounds([0.,1.,0.],[.1,1.1,.1])
grid.resize(32,32,32)
for idx in range(grid.dims[0]):
    for idy in range(grid.dims[1]):
        for idz in range(grid.dims[2]):
            d=[(idx-grid.dims[0]/2)/float(grid.dims[0]/2),  \
               (idy-grid.dims[1]/2)/float(grid.dims[1]/2),  \
               (idz-grid.dims[2]/2)/float(grid.dims[2]/2)]
            freq=2.
            amp=.1
            thres=.5*(1.+math.sin(d[0]*math.pi*freq)*math.sin(d[1]*math.pi*freq)*math.sin(d[2]*math.pi*freq)*amp)
            grid.set(idx,idy,idz,op.norm(d)/op.norm([1.,1.,1.])-thres)
sdf.geometry().setVolumeGrid(grid)
sdf.appearance().setColor(random.uniform(0.,1.),random.uniform(0.,1.),random.uniform(0.,1.))

#example PointCloud
points=world.makeTerrain("PointCloud")
cloud=PointCloud()
for i in range(1000):
    cloud.addPoint([random.uniform(0.,0.1),random.uniform(0.,0.1),random.uniform(0.,0.1)])
points.geometry().setPointCloud(cloud)
points.geometry().transform([1,0,0,0,1,0,0,0,1],[0.,1.2,0.])
points.appearance().setColor(random.uniform(0.,1.),random.uniform(0.,1.),random.uniform(0.,1.))

#robot
robot=world.loadRobot("../../data/robots/tx90robotiq.rob")
for l in range(robot.numLinks()):
    lk=robot.link(l)
    t=lk.getTransform()
    t=se3.mul(([.1,0,0,0,.1,0,0,0,.1],[0.,0.,0.]),t)
    t=se3.mul(([1,0,0,0,1,0,0,0,1],[0.05,1.5,0.]),t)
    lk.setTransform(t[0],t[1])
    lk.appearance().setColor(random.uniform(0.,1.),random.uniform(0.,1.),random.uniform(0.,1.))

class GLVisualizer(GLProgram):
    def __init__(self,world):
        GLProgram.__init__(self,"Visualizer")
        self.zeroZ=True
        self.world=world
        self.init_camera()
        self.properties={}

    def look_at(self,pos,tgt,scale=None):
        cam=self.view.camera
        if scale is not None:
            cam.dist=op.norm(op.sub(tgt,pos))*scale
        cam.rot=self.get_camera_rot(op.sub(pos,tgt))
        cam.tgt=tgt
        
    def get_camera_pos(self):
        cam=self.view.camera
        z=math.sin(-cam.rot[1])
        x=math.sin(cam.rot[2])*math.cos(cam.rot[1])
        y=math.cos(cam.rot[2])*math.cos(cam.rot[1])
        pos=[x,y,z]
        return op.add(cam.tgt,op.mul(pos,cam.dist))
    
    def get_camera_rot(self,d):
        angz=math.atan2(d[0],d[1])
        angy=math.atan2(-d[2],math.sqrt(d[0]*d[0]+d[1]*d[1]))
        return [0,angy,angz]

    def get_camera_dir(self,zeroZ=False):
        cam=self.view.camera
        dir=op.sub(cam.tgt,self.get_camera_pos())
        if zeroZ:
            dir=(dir[0],dir[1],0)
        if op.norm(dir)>1e-6:
            dir=op.mul(dir,1/op.norm(dir))
        dir[1]*=-1
        return dir

    def get_left_dir(self,zeroZ=False):
        dir=op.cross([0,0,1],self.get_camera_dir())
        if zeroZ:
            dir=(dir[0],dir[1],0)
        if op.norm(dir)>1e-6:
            dir=op.mul(dir,1/op.norm(dir))
        return dir

    def get_bb(self):
        bb=None
        for i in range(world.numTerrains()):
            bb=self.union_bb(bb,world.terrain(i).geometry().getBBTight())
        for i in range(world.numRigidObjects()):
            bb=self.union_bb(bb,world.rigidObject(i).geometry().getBBTight())
        for r in range(world.numRobots()):
            for l in range(world.robot(r).numLinks()):
                bb=self.union_bb(bb,world.robot(r).link(l).geometry().getBBTight())
        return bb

    def init_camera(self):
        bb=self.get_bb()
        pos=list(bb[1])
        tgt=list(bb[0])
        
        self.look_at(pos,tgt,2.0)
        self.moveSpd=0.005
        self.zoomSpd=1.03
        self.zoomMin=0.01
        self.zoomMax=100.0
        
        self.zoomInCam=False
        self.zoomOutCam=False
        self.forwardCam=False
        self.backCam=False
        self.leftCam=False
        self.rightCam=False
        self.raiseCam=False
        self.sinkCam=False

    def keyboardfunc(self,c,x,y):
        if c==b'f':
            self.init_camera()
        elif c==b'z':
            self.zeroZ=not self.zeroZ
        elif c==b'q':
            self.zoomInCam=True
        elif c==b'e':
            self.zoomOutCam=True
        elif c==b'w':
            self.forwardCam=True
        elif c==b's':
            self.backCam=True
        elif c==b'a':
            self.leftCam=True
        elif c==b'd':
            self.rightCam=True
        elif c==b' ':
            self.raiseCam=True
        elif c==b'c':
            self.sinkCam=True

    def keyboardupfunc(self,c,x,y):
        if c==b'q':
            self.zoomInCam=False
        elif c==b'e':
            self.zoomOutCam=False
        elif c==b'w':
            self.forwardCam=False
        elif c==b's':
            self.backCam=False
        elif c==b'a':
            self.leftCam=False
        elif c==b'd':
            self.rightCam=False
        elif c==b' ':
            self.raiseCam=False
        elif c==b'c':
            self.sinkCam=False
        elif c==b'v':
            #if self.world.numRobots()>0:
            #    self.world.robot(0).setConfig([random.uniform(-1.,1.) for d in range(self.world.robot(0).numLinks())])
            povray.to_povray(self,self.world,self.properties)
            self.save_screen('screenshot-klampt.png')
            print('rendered!')
    
    def display_screen(self):
        self.draw_text((0,12),'press v to invoke renderer!',color=(0,0,0))
    
    def display(self):
        self.world.drawGL()

    def handle_camera(self):
        self.view.clippingplanes=(self.view.clippingplanes[0],self.zoomMax)
        cam=self.view.camera
        moveSpd=self.moveSpd*cam.dist
        if self.zoomInCam:
            cam.dist=max(cam.dist/self.zoomSpd,self.zoomMin)
        elif self.zoomOutCam:
            cam.dist=min(cam.dist*self.zoomSpd,self.zoomMax)
        elif self.forwardCam:
            delta=op.mul(self.get_camera_dir(self.zeroZ),moveSpd)
            self.look_at(op.add(self.get_camera_pos(),delta),op.add(cam.tgt,delta))
        elif self.backCam:
            delta=op.mul(self.get_camera_dir(self.zeroZ),-moveSpd)
            self.look_at(op.add(self.get_camera_pos(),delta),op.add(cam.tgt,delta))
        elif self.leftCam:
            delta=op.mul(self.get_left_dir(self.zeroZ),moveSpd)
            self.look_at(op.add(self.get_camera_pos(),delta),op.add(cam.tgt,delta))
        elif self.rightCam:
            delta=op.mul(self.get_left_dir(self.zeroZ),-moveSpd)
            self.look_at(op.add(self.get_camera_pos(),delta),op.add(cam.tgt,delta))
        elif self.raiseCam:
            delta=(0,0,moveSpd)
            self.look_at(op.add(self.get_camera_pos(),delta),op.add(cam.tgt,delta))
        elif self.sinkCam:
            delta=(0,0,-moveSpd)
            self.look_at(op.add(self.get_camera_pos(),delta),op.add(cam.tgt,delta))

    def idle(self):
        self.handle_camera()
        
    def union_bb(self,bb1,bb2):
        if bb1 is None:
            return bb2
        elif bb2 is None:
            return bb1
        else:
            return ([min(a,b) for a,b in zip(bb1[0],bb2[0])],[max(a,b) for a,b in zip(bb1[1],bb2[1])])

vis=GLVisualizer(world)
vis.properties["tempfile"]="tmpPovray/__temp__.pov"
vis.properties["outfile"]="screenshot.png"
#set default parameter of radius
vis.properties["radius"]=0.01
#set a different radius for world.terrain("PointCloud")
vis.properties["PointCloud"]={"radius":0.001}   
#locate lights according to bounding box of the world, you can also set it by yourself
pos,tgt=povray.create_env_light_for_bb(vis.get_bb())   
#add lights, turn it to pointlight by setting spotlight=False, area>0 means this is area light with soft shadow
povray.add_light(vis.properties,pos,tgt,spotlight=True,area=.1,color=[2.,2.,2.])   
#change the material of floor to stone
vis.properties["Floor"]={"hide":False}
vis.properties["Floor"]["finish"]=vp.Finish('ambient',0.,'diffuse',.5,'specular',.15)
vis.properties["Floor"]["normal"]=vp.Normal('granite',0.2,'warp {turbulence 1}','scale',.25)
#change the material of VolumeGrid to metal
vis.properties["SDF"]={}
preset=False
if preset:
    vis.properties["SDF"]["finish"]=vp.Finish('F_MetalB')
    #F_MetalB is a predefined material in povray, you have to include 'metal.inc' to use it
    vis.properties["included"]=['metals.inc']
else:
    vis.properties["SDF"]["finish"]=vp.Finish('ambient',0.30,'brilliance',3,'diffuse',0.4,'metallic','specular',0.70,'roughness',1/60.,'reflection',.25)
for l in range(robot.numLinks()):
    name=robot.link(l).getName()
    vis.properties[name]={"transient":False}
vis.run()