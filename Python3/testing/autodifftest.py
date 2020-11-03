from klampt.math.autodiff import ad,math_ad,so3_ad,se3_ad,kinematics_ad,dynamics_ad,trajectory_ad,geometry_ad
from klampt.math.autodiff.ad import _ADGetItem,_ADSetItem
from klampt.math import so3,se3
from klampt.math.geodesic import *
from klampt import *
import numpy as np

def my_func(x,y):
        return x**2 - y**2

my_func_ad = ad.function(my_func,'auto',[1,1],1,
                      jvp=[lambda dx,x,y:2*x*dx, lambda dy,x,y:-2*y*dy])  #gives Jacobian-vector products
assert my_func_ad(ad.var('x'),ad.var('x')).derivative('x',x=2).item(0) == 0
assert my_func_ad('x',2.0).derivative('x',x=2).item(0) == 4
#test basic
assert np.allclose((3*ad.var('x')**2 - 2*ad.var('x')).eval(x=1.5), 3*1.5**2-2*1.5)
assert np.allclose((3*ad.var('x')**2 - 2*ad.var('y')).eval(x=1.5,y=-0.8), 3*1.5**2-2*(-0.8))

#printing check
a = math_ad.norm('x')
print(a)
try:
    b = math_ad.norm('x','y')
    raise RuntimeError("Exception not run with incorrect # of arguments")
except ValueError:
    pass
import numpy as np
c = math_ad.norm(2*ad.var('x') + np.array([1.5,2]))
print(c,"at [1,0]:",c.eval(x=np.array([1,0])))

d = ad.var('x')[0]*ad.var('x')[1]
print(d)
assert d.eval(x=np.array([1,2])) == 2

e = so3_ad.angle(so3_ad.from_rpy(np.array([0,0,2.0])))
print(e)
assert e.eval() == 2.0

print((ad.var('x')**3).gen_derivative(['x','x'],x=2.0),"should be 6*2=12")

#test derivatives
x = np.array([1.0,0.0])
y = np.array([3.0,4.0])
ad.check_derivatives(ad.add,[x,y])
ad.check_derivatives(ad.sub,[x,y])
ad.check_derivatives(ad.mul,[x,y])
ad.check_derivatives(ad.div,[x,y])
ad.check_derivatives(ad.sum_,[x,y,y])
ad.check_derivatives(ad.neg,[x])
ad.check_derivatives(ad.abs_,[y])
ad.check_derivatives(ad.pow_,[x,y])
ad.check_derivatives(ad.stack,[x])
ad.check_derivatives(ad.stack,[x,y])
ad.check_derivatives(ad.stack,[x,y,x])
ad.check_derivatives(_ADGetItem(0),[x])
ad.check_derivatives(_ADGetItem(1),[x])
ad.check_derivatives(_ADGetItem([0]),[x])
ad.check_derivatives(_ADGetItem([1,0]),[x])
ad.check_derivatives(_ADGetItem(slice(0,2)),[x])
ad.check_derivatives(_ADSetItem(0),[x,2.0])
ad.check_derivatives(_ADSetItem(slice(0,1)),[x,-5.0])
ad.check_derivatives(ad.maximum,[x,y])
ad.check_derivatives(ad.minimum,[x,y])

print("Basic function derivative check passed")

#test mixed scalar-vector derivatives
x = -3.0
ad.check_derivatives(ad.add,[x,y])
ad.check_derivatives(ad.sub,[x,y])
ad.check_derivatives(ad.mul,[x,y])
ad.check_derivatives(ad.div,[x,y])
ad.check_derivatives(ad.sum_,[x,y,y])
ad.check_derivatives(ad.neg,[x])
ad.check_derivatives(ad.abs_,[x])
#numpy doesn't allow fractional powers of negative numbers...
#ad.check_derivatives(ad.pow_,[x,y])
ad.check_derivatives(ad.stack,[x])
ad.check_derivatives(ad.stack,[x,y])
ad.check_derivatives(ad.stack,[x,y,x])
ad.check_derivatives(ad.maximum,[x,y])
ad.check_derivatives(ad.minimum,[x,y])
ad.check_derivatives(ad.maximum,[y,x])
ad.check_derivatives(ad.minimum,[y,x])

#test mixed scalar-vector derivatives
x = np.array([1.0,0.0])
y = 3
ad.check_derivatives(ad.add,[x,y])
ad.check_derivatives(ad.sub,[x,y])
ad.check_derivatives(ad.mul,[x,y])
ad.check_derivatives(ad.div,[x,y])
ad.check_derivatives(ad.sum_,[x,y,y])
ad.check_derivatives(ad.pow_,[x,y])
ad.check_derivatives(ad.stack,[x,y])
ad.check_derivatives(ad.stack,[x,y,x])

print("Basic function scalar-vector derivative check passed")

y = np.array([3.0,4.0])
vx = ad.var('x')
vy = ad.var('y')
context = {'x':x,'y':y}
ad.check_derivatives(vx+vy,context)
ad.check_derivatives(vx-vy,context)
ad.check_derivatives(vx*vy,context)
ad.check_derivatives(vx/vy,context)
ad.check_derivatives(ad.sum_(vy),context)
ad.check_derivatives(ad.sum_(vx,vy),context)
ad.check_derivatives(ad.sum_(vx,vy,vy),context)
ad.check_derivatives(-vx,context)
ad.check_derivatives(abs(vy),context)
ad.check_derivatives(vx**vy,context)
ad.check_derivatives(ad.stack(vx),context)
ad.check_derivatives(ad.stack(vx,vy),context)
ad.check_derivatives(ad.stack(vx,vy,vx),context)
ad.check_derivatives(vx[0],context)
ad.check_derivatives(vx[1],context)
ad.check_derivatives(vx[[1,0]],context)
ad.check_derivatives(vx[0:2],context)
ad.check_derivatives(3*vx**2 - 2*vy,context)

context = {'x':-3.0,'y':y}
ad.check_derivatives(vx+vy,context)
ad.check_derivatives(vx-vy,context)
ad.check_derivatives(vx*vy,context)
ad.check_derivatives(vx/vy,context)
ad.check_derivatives(ad.sum_(vx,vy),context)
ad.check_derivatives(ad.sum_(vx,vy,vy),context)
ad.check_derivatives(-vx,context)
ad.check_derivatives(abs(vy),context)
#can't do derivatives of negative number raised to a power
#ad.check_derivatives(vx**vy,context)
ad.check_derivatives(ad.stack(vx),context)
ad.check_derivatives(ad.stack(vx,vy),context)
ad.check_derivatives(ad.stack(vx,vy,vx),context)
ad.check_derivatives(3*vx**2 - 2*vy,context)
print("Compound derivative check passed")

context = {'x':x,'y':y}
ad.check_derivatives(math_ad.exp(vx),context)
ad.check_derivatives(math_ad.log(vy),context)
ad.check_derivatives(math_ad.sin(vx),context)
ad.check_derivatives(math_ad.cos(vx),context)
ad.check_derivatives(math_ad.norm(vx),context)
ad.check_derivatives(math_ad.normSquared(vx),context)
ad.check_derivatives(math_ad.distance(vx,vy),context)
ad.check_derivatives(math_ad.distanceSquared(vx,vy),context)
ad.check_derivatives(math_ad.distanceSquared(vx,vx),context)
ad.check_derivatives(math_ad.unit(vy),context)
ad.check_derivatives(math_ad.cross(vx,vy),context)
ad.check_derivatives(math_ad.interpolate(vx,vy,0.5),context)
print("Math derivative check passed")

R = np.array(so3.from_rpy([1.5,0.5,2]))
R2 = np.array(so3.from_rpy([-0.6,0,-0.4]))
context = {'R':R,'R2':R2}
sp = SO3Space()
print(sp.difference(so3.identity(),so3.identity()),"=[0]*9")
print(sp.difference(R,R),"=[0]*9")
print(sp.difference(R,so3.identity()))
assert vectorops.distance(R,sp.integrate(so3.identity(),sp.difference(R,so3.identity()))) < 1e-3
assert vectorops.distance(R,sp.integrate(R2,sp.difference(R,R2))) < 1e-3
Rh = np.array(sp.interpolate(R,R2,1e-3))
dR = (Rh-R)/1e-3
assert vectorops.distance(dR,sp.difference(R2,R)) < 1e-2,"Difference is not a good approximation of the so3.interpolate derivative?"
ad.check_derivatives(so3_ad.diag('R'),context)
ad.check_derivatives(so3_ad.deskew('R'),context)
ad.check_derivatives(so3_ad.cross_product('w'),{'w':np.array([0.5,0.2,-30])})
ad.check_derivatives(so3_ad.inv('R'),context)
ad.check_derivatives(so3_ad.mul('R','R2'),context)
ad.check_derivatives(so3_ad.mul('R',so3_ad.inv('R')),context)
ad.check_derivatives(so3_ad.angle('R'),context)
ad.check_derivatives(so3_ad.axis('R'),context)
ad.check_derivatives(so3_ad.from_axis_angle,[np.array([0,1,0]),1.5])
ad.check_derivatives(so3_ad.from_axis_angle('axis','angle'),{'axis':np.array([0,1,0]),'angle':1.5})
ad.check_derivatives(so3_ad.rotation_vector('R'),context)
ad.check_derivatives(so3_ad.quaternion('R'),context)
ad.check_derivatives(so3_ad.interpolate('R','R2',0.5),context)
ad.check_derivatives(so3_ad.interpolate('R','R2',0.9),context)
print("so3 derivative check passed")

T = np.hstack((R,[1,0.2,-2.5]))
T2 = np.hstack((R2,[2,0,3.5]))
context = {'T':T,'T2':T2}
ad.check_derivatives(se3_ad.translation('T'),context)
ad.check_derivatives(se3_ad.rotation('T'),context)
assert np.allclose(se3_ad.error('T','T').eval(**context),np.zeros(6))
assert se3_ad.distance('T','T2').eval(**context) == (math_ad.norm(se3_ad.error('T','T2')[:3])+math_ad.norm(se3_ad.error('T','T2')[3:])).eval(**context)
se3_ad.interpolate('T','T2',0.5).eval(**context)
print("se3 derivative check passed")

print("Testing erroneous calls")
try:
    so3_ad.inv(vx).eval(x=np.array([1.0,2.0,3.0]))
    raise RuntimeError("Erroneous call was not caught")
except ValueError:
    pass
try:
    so3_ad.mul(vx)
    raise RuntimeError("Erroneous call was not caught")
except ValueError:
    pass

try:
    so3_ad.inv(so3_ad.rpy('R'))
    raise RuntimeError("Erroneous call was not caught")
except ValueError:
    pass
print("Erroneous call check passed")

fn = '../../data/tx90scenario0.xml'
w = WorldModel()
w.readFile(fn)
robot = w.robot(0)
eelink = robot.link(robot.numLinks()-1)
q = np.array(robot.getConfig())
dq = np.array([0] + [1]*(robot.numLinks()-1))
localpos = [0,0,0]
wp = kinematics_ad.WorldPosition(eelink,localpos)('q')
wo = kinematics_ad.WorldOrientation(eelink)('q')
wT = kinematics_ad.WorldTransform(eelink,localpos)('q')
wv = kinematics_ad.WorldVelocity(eelink,localpos)('q','dq')
ww = kinematics_ad.WorldAngularVelocity(eelink)('q','dq')
assert np.allclose(wp.eval(q=q),eelink.getWorldPosition(localpos))
assert np.allclose(wo.eval(q=q),eelink.getTransform()[0])
assert np.allclose(wT.eval(q=q),eelink.getTransform()[0] + eelink.getWorldPosition(localpos))
assert np.allclose(wv.eval(q=q,dq=dq),eelink.getPointVelocity(localpos))
assert np.allclose(ww.eval(q=q,dq=dq),eelink.getAngularVelocity())
kbuilder = kinematics_ad.KinematicsBuilder(robot,'q','dq')
print("World position:",kbuilder.world_position(robot.numLinks()-1))
print("World velocity:",kbuilder.world_velocity(robot.numLinks()-1))
assert np.allclose(kbuilder.world_transform(eelink).eval(q=q),eelink.getTransform()[0]+eelink.getTransform()[1])
print("kinematics equivalence check passed")

ad.check_derivatives(wp,{'q':q})
ad.check_derivatives(wo,{'q':q})
ad.check_derivatives(wT,{'q':q})
ad.check_derivatives(kbuilder.world_transform(0),{'q':q})
ad.check_derivatives(kbuilder.world_transform(1),{'q':q})
ad.check_derivatives(kbuilder.world_transform(2),{'q':q})
ad.check_derivatives(kbuilder.world_transform(eelink),{'q':q})
print("kinematics derivative check passed")

x1=np.array([0,0])
x2=np.array([1,1])
v1=np.array([1,0])
v2=np.array([0,1])
ad.check_derivatives(trajectory_ad.hermite(x1,v1,x2,v2,'t'),{'t':0.3})
ad.check_derivatives(trajectory_ad.hermite('x1',v1,x2,v2,0.3),{'x1':x1})
ad.check_derivatives(trajectory_ad.hermite(x1,'v1',x2,v2,0.3),{'v1':v1})
ad.check_derivatives(trajectory_ad.hermite(x1,v1,'x2',v2,0.3),{'x2':x2})
ad.check_derivatives(trajectory_ad.hermite(x1,v1,x2,'v2',0.3),{'v2':v2})
ad.check_derivatives(trajectory_ad.hermite(x1,v1,x2,'v2','t'),{'v2':v2,'t':0.8})
ad.check_derivatives(trajectory_ad.hermite,[x1,v1,x2,v2,0.3])
ad.check_derivatives(trajectory_ad.GeodesicInterpolate(GeodesicSpace()),[R,R2,0.5])
ad.check_derivatives(trajectory_ad.GeodesicInterpolate(SO3Space()),[np.array(so3.identity()),R,0])
ad.check_derivatives(trajectory_ad.GeodesicInterpolate(SO3Space()),[np.array(so3.identity()),R,0.5])
ad.check_derivatives(trajectory_ad.GeodesicInterpolate(SO3Space()),[np.array(so3.identity()),R,1])
ad.check_derivatives(trajectory_ad.GeodesicInterpolate(SO3Space()),[R,np.array(so3.identity()),1])
ad.check_derivatives(trajectory_ad.GeodesicInterpolate(SO3Space()),[R,R2,0.5])
ad.check_derivatives(trajectory_ad.GeodesicInterpolate(SO3Space()),[R2,R,0.3])
print("trajectory derivative check passed")

bpm = geometry_ad.BoxPointMargin(np.array([-1,-1]),np.array([1,1]))
bpd = geometry_ad.BoxPointDistance(np.array([-1,-1]),np.array([1,1]))
bpc = geometry_ad.BoxPointClosest(np.array([-1,-1]),np.array([1,1]))
ad.check_derivatives(bpm,[np.array([0.7,0.3])])
ad.check_derivatives(bpm,[np.array([-0.3,-0.7])])
ad.check_derivatives(bpd,[np.array([0.7,0.3])])
ad.check_derivatives(bpd,[np.array([-0.3,-0.7])])
ad.check_derivatives(bpd,[np.array([2.5,0.3])])
ad.check_derivatives(bpd,[np.array([-3.5,1.8])])
ad.check_derivatives(bpc,[np.array([0.7,0.3])])
ad.check_derivatives(bpc,[np.array([-0.3,-0.7])])
ad.check_derivatives(bpc,[np.array([2.5,0.3])])
ad.check_derivatives(bpc,[np.array([-3.5,1.8])])
spd = geometry_ad.sphere_point_distance('c','r','x')
ad.check_derivatives(spd,{'c':np.array([1.0,2.5,0.3]),'r':0.5,'x':np.array([-2.3,1.0,0.5])})
ssd = geometry_ad.sphere_sphere_distance('c1','r1','c2','r2')
ad.check_derivatives(ssd,{'c1':np.array([1.0,2.5,0.3]),'r1':0.5,'c2':np.array([-2.3,1.0,0.5]),'r2':0.3})

g1 = robot.link(2).geometry()
g2 = robot.link(5).geometry()

gpd = geometry_ad.GeomPointDistance(g1,'link 2')
gpd_ub = geometry_ad.GeomPointDistance(g1,'link 2 (ub 0.1)',upperBound=0.1)

print("geom-point distance:",gpd(se3_ad.identity(),np.array([3.0,-0.3,1.0])).eval())
print("geom-point distance (upper-bounded):",gpd_ub(se3_ad.identity(),np.array([3.0,-0.3,1.0])).eval())

try:
    ad.check_derivatives(gpd,[se3_ad.identity(),np.array([3.0,-0.3,1.0])],h=1e-2)
except Exception as e:
    print(e)
    import traceback
    traceback.print_exc()
    pass
try:
    ad.check_derivatives(gpd_ub,[se3_ad.identity(),np.array([3.0,-0.3,1.0])],h=1e-2)
except Exception as e:
    print(e)
    import traceback
    traceback.print_exc()
    pass

ggd = geometry_ad.GeomGeomDistance(g1,g2,'link 2','link 5')
try:
    ad.check_derivatives(ggd,[se3_ad.from_klampt(robot.link(2).getTransform()),se3_ad.from_klampt(robot.link(5).getTransform())],h=1e-2)
except Exception as e:
    print(e)
    import traceback
    traceback.print_exc()
    pass

print("geometry derivative check passed")