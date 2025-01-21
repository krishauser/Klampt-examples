from klampt.math.geodesic import SO3Space,SE3Space
from klampt.math import so2,so3,se3,vectorops
import numpy as np

def _test_small(name,value,*args,tol=1e-4):
    if abs(value) > tol:
        print(name,"failed, value",value)
        for i in range(0,len(args),2):
            print("   ",args[i],args[i+1])

def test_so3():
    a = so3.from_rpy((0,0,0))
    b = so3.from_rpy((-1.5,0.5,2))
    c = so3.rotation([0,0,1],np.pi)
    d = so3.rotation([1,0,0],np.pi)
    e = so3.mul(b,d)
    _test_small('RPY conversion',vectorops.distance(so3.from_rpy(so3.rpy(a)),a))
    _test_small('RPY conversion',vectorops.distance(so3.from_rpy(so3.rpy(b)),b))
    _test_small('RPY conversion',vectorops.distance(so3.from_rpy(so3.rpy(c)),c))
    _test_small('RPY conversion',vectorops.distance(so3.from_rpy(so3.rpy(d)),d))
    _test_small('RPY conversion',vectorops.distance(so3.from_rpy(so3.rpy(e)),e))
    _test_small('Rotation vector conversion',vectorops.distance(so3.from_rotation_vector(so3.rotation_vector(a)),a))
    _test_small('Rotation vector conversion',vectorops.distance(so3.from_rotation_vector(so3.rotation_vector(b)),b))
    _test_small('Rotation vector conversion',vectorops.distance(so3.from_rotation_vector(so3.rotation_vector(c)),c))
    _test_small('Rotation vector conversion',vectorops.distance(so3.from_rotation_vector(so3.rotation_vector(d)),d))
    _test_small('Rotation vector conversion',vectorops.distance(so3.from_rotation_vector(so3.rotation_vector(e)),e))
    _test_small('Quaternion conversion',vectorops.distance(so3.from_quaternion(so3.quaternion(a)),a))
    _test_small('Quaternion conversion',vectorops.distance(so3.from_quaternion(so3.quaternion(b)),b))
    _test_small('Quaternion conversion',vectorops.distance(so3.from_quaternion(so3.quaternion(c)),c))
    _test_small('Quaternion conversion',vectorops.distance(so3.from_quaternion(so3.quaternion(d)),d))
    _test_small('Quaternion conversion',vectorops.distance(so3.from_quaternion(so3.quaternion(e)),e))
    _test_small('Interpolate',vectorops.distance(so3.interpolate(b,c,0),b))
    _test_small('Interpolate',vectorops.distance(so3.interpolate(b,c,1),c))
    h = 1e-5
    dr = vectorops.div(vectorops.sub(so3.interpolate(c,b,h),so3.interpolate(c,b,0)),h)
    w = so3.error(b,c)
    dr_est = so3.mul(so3.cross_product(w),c)
    _test_small('Error claim holds',vectorops.distance(dr,dr_est))

def test_geodesic():
    s = SO3Space()
    a = so3.from_rpy((0,0,0))
    b = so3.from_rpy((-1.5,0.5,2))
    d = s.difference(a,b)
    a2 = s.integrate(b,d)
    _test_small('SO3 integrate difference from zero',vectorops.distance(a,a2))
    s3 = SE3Space()
    a = a + [0.0,1.0,2.0]
    b = b + [1.0,0.0,0.0]
    d = s3.difference(a,b)
    a2 = s3.integrate(b,d)
    _test_small('SO3 integrate difference',vectorops.distance(a,a2))
    a = (1,3,2)
    b = (-0.5,0.4,1.5)
    R = so3.align(a,b)
    b2 = so3.apply(R,a)
    _test_small('SO3 align',vectorops.distance(vectorops.unit(b),vectorops.unit(b2)))
    a = (1,0,0)
    b = (-2,0,0)
    R = so3.align(a,b)
    b2 = so3.apply(R,a)
    _test_small('SO3 align opposed',vectorops.distance(vectorops.unit(b),vectorops.unit(b2)))


if __name__ == '__main__':
    test_so3()
    test_geodesic()
    print("All tests passed")
