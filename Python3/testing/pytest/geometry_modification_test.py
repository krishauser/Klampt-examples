from klampt import WorldModel,Geometry3D, TriangleMesh, PointCloud, GeometricPrimitive
from klampt.math import se3,so3
from klampt.model.create import box,sphere
import pytest

@pytest.fixture
def primitive_geom_standalone() -> Geometry3D:
    g = box(1,2,3,type='GeometricPrimitive')
    return g

@pytest.fixture
def trimesh_geom_standalone() -> Geometry3D:
    g = box(1,2,3,type='TriangleMesh')
    return g

@pytest.fixture
def world() -> WorldModel:
    return WorldModel()

@pytest.fixture
def primitive_geom_ref(world,primitive_geom_standalone) -> Geometry3D:
    o = world.makeRigidObject('primitive')
    o.geometry().set(primitive_geom_standalone)
    assert isinstance(primitive_geom_standalone, Geometry3D)
    return o.geometry()

@pytest.fixture
def trimesh_geom_ref(world,trimesh_geom_standalone) -> Geometry3D:
    o = world.makeRigidObject('trimesh')
    o.geometry().set(trimesh_geom_standalone)
    assert isinstance(trimesh_geom_standalone, Geometry3D)
    return o.geometry()

@pytest.fixture(params=[0,1])
def primitive_geom(primitive_geom_standalone,primitive_geom_ref,request) -> Geometry3D:
    if request.param == 0:
        return primitive_geom_standalone
    else:
        return primitive_geom_ref

@pytest.fixture(params=[0,1])
def trimesh_geom(trimesh_geom_standalone,trimesh_geom_ref,request) -> Geometry3D:
    if request.param == 0:
        return trimesh_geom_standalone
    else:
        return trimesh_geom_ref

def test_copy(primitive_geom):
    assert primitive_geom.type() == 'GeometricPrimitive'
    g = Geometry3D(primitive_geom)
    assert g.type() == 'GeometricPrimitive'
    assert primitive_geom.copy().type() == 'GeometricPrimitive'

def test_extract(primitive_geom):
    prim = primitive_geom.getGeometricPrimitive()
    g = Geometry3D(prim)
    with pytest.raises(Exception) as e_info:
        trimesh = primitive_geom.getTriangleMesh()

def test_extract_create(primitive_geom):
    prim = primitive_geom.getGeometricPrimitive()
    g = Geometry3D(prim)
    assert g.type() == 'GeometricPrimitive'

def test_get_set(primitive_geom):
    prim = primitive_geom.getGeometricPrimitive()
    g = Geometry3D()
    g.setGeometricPrimitive(prim)
    prim2 = g.getGeometricPrimitive()
    primitive_geom.setGeometricPrimitive(prim2)

def test_set_change1(primitive_geom,trimesh_geom):
    #changes a geometry type to a different data via setX
    g = Geometry3D(primitive_geom.copy())
    g.setTriangleMesh(trimesh_geom.getTriangleMesh())
    assert primitive_geom.type() == 'GeometricPrimitive'
    assert g.type() == 'TriangleMesh'
    g.setCurrentTransform(so3.rotation([0,0,1],0.1),[1,2,3])
    assert primitive_geom.getCurrentTransform() == (so3.identity(),[0,0,0])
    g.free()
    assert primitive_geom.type() == 'GeometricPrimitive'

def test_set_change2(primitive_geom,trimesh_geom):
    #changes a geometry type to a different data via set(Geometry3D)
    g = Geometry3D(primitive_geom.copy())
    g.set(trimesh_geom)
    assert primitive_geom.type() == 'GeometricPrimitive'
    assert g.type() == 'TriangleMesh'
    assert trimesh_geom.type() == 'TriangleMesh'
    g.setCurrentTransform(so3.rotation([0,0,1],0.1),[1,2,3])
    assert primitive_geom.getCurrentTransform() == (so3.identity(),[0,0,0])
    g.free()
    assert primitive_geom.type() == 'GeometricPrimitive'
    assert trimesh_geom.type() == 'TriangleMesh'

def test_convert(primitive_geom,trimesh_geom):
    bb = primitive_geom.getBBTight()
    gtri = primitive_geom.convert('TriangleMesh')
    assert gtri.type() == 'TriangleMesh'
    assert primitive_geom.type() == 'GeometricPrimitive'
    gtri.setCurrentTransform(so3.rotation([0,0,1],0.1),[1,2,3])
    bb = gtri.getBBTight()
    pcgeom = gtri.convert('PointCloud')
    assert pcgeom.type() == 'PointCloud'
    assert gtri.type() == 'TriangleMesh'
    pcgeom.setCurrentTransform(so3.rotation([0,0,1],0.1),[1,2,3])
    bb = gtri.getBBTight()
    pcgeom = trimesh_geom.convert('PointCloud')
    assert pcgeom.type() == 'PointCloud'
    assert trimesh_geom.type() == 'TriangleMesh'
    pcgeom.setCurrentTransform(so3.rotation([0,0,1],0.1),[1,2,3])


def test_failure():
    return
    a = sphere(0.4,center=(0,0,0),type='TriangleMesh')
    a = Geometry3D(a)

    w = WorldModel()
    o = w.makeRigidObject("a")
    #w.rigidObject(0).geometry().set(a)
    o.geometry().set(a)

    #a_pc = a.convert("PointCloud",0.05)
    bb = a.getBBTight()
    a_pc = a.convert("PointCloud")
    a_pc.setCurrentTransform(so3.identity(),[1.25*(bb[1][0]-bb[0][0]),0,0])