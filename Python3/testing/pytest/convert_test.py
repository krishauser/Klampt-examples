from pytest import fixture
from klampt import *
from klampt.io.numpy_convert import *
from klampt.model.trajectory import Trajectory,RobotTrajectory
from klampt.io import resource

@fixture
def world():
    w = WorldModel()
    assert w.loadFile("../../../data/athlete_fractal_1.xml")
    return w

@fixture
def traj(world):
    resource.set_directory('.')
    traj = resource.get("../../../data/motions/athlete_flex_opt.path")
    assert traj is not None
    return traj

def test_convert_pc(world):
    geom = world.terrain(0).geometry()
    pc = geom.convert('PointCloud').getPointCloud()
    pcnp = to_numpy(pc)
    pc2 = from_numpy(pcnp,'PointCloud')
    assert len(pc2.points) == len(pc.points)
    assert len(pc2.properties) == len(pc.properties)

def test_convert_mesh(world):
    geom = world.terrain(0).geometry()
    mesh = geom.getTriangleMesh()
    mnp = to_numpy(mesh)
    m2 = from_numpy(mnp,'TriangleMesh')
    assert len(m2.vertices) == len(mesh.vertices)
    assert len(m2.indices) == len(mesh.indices)

def test_convert_volumegrid(world):
    geom2 = world.robot(0).link(11).geometry()
    vg = geom2.convert('VolumeGrid',0.01)
    vgnp = to_numpy(vg.getVolumeGrid())
    assert len(vgnp) == 3, "VolumeGrid should have 3 elements, instead got {}".format(len(vgnp))
    assert len(vgnp[0]) == 3
    assert len(vgnp[1]) == 3

def test_convert_geom(world):
    geom = world.terrain(0).geometry()
    gnp = to_numpy(geom)
    assert len(gnp) == 2, "Geometry should have 2 elements"
    assert gnp[0].shape == (4,4), "First element should be a 4x4 matrix"
    g2 = from_numpy(gnp,'Geometry3D')
    assert g2.type() == geom.type()

def test_convert_traj(traj,world):
    trajnp = to_numpy(traj)
    traj2 = from_numpy(trajnp,template=traj)
    assert traj2.__class__.__name__ == "Trajectory", "Return type should be Trajectory"
    rtraj = RobotTrajectory(world.robot(0),traj.times,traj.milestones)
    rtrajnp = to_numpy(rtraj)
    rtraj2 = from_numpy(rtrajnp,template=rtraj)
    assert rtraj2.__class__.__name__ == "RobotTrajectory", "Return type should be RobotTrajectory"

def test_open3d(world):
    try:
        from klampt.io.open3d_convert import from_open3d,to_open3d
        import open3d
    except ImportError:
        print("It looks like open3D isn't installed, can't run this test")
        raise 
    geom2 = world.robot(0).link(11).geometry()
    vg = geom2.convert('VolumeGrid',0.01)
    assert len(vg.getVolumeGrid().getValues()) > 0

    omesh = to_open3d(geom2.getTriangleMesh())
    omesh.compute_vertex_normals()
    ogrid = to_open3d(vg.getVolumeGrid())
    open3d.visualization.draw_geometries([omesh,ogrid])

    vmesh = from_open3d(omesh)
    vgrid = from_open3d(ogrid)
    assert len(vgrid.getValues()) > 0,"Conversion from open3d gave a VolumeGrid with no values"

    from klampt import vis
    # vis.add("grid",Geometry3D(vgrid))
    # vis.setColor("grid",0,1,0)
    vis.add("mesh",Geometry3D(vmesh))
    vis.spin(2.0)

    geom = world.terrain(0).geometry()
    pc = geom.convert('PointCloud').getPointCloud()
    opc = to_open3d(pc)
    vpc = from_open3d(opc)

    if open3d.__version__ >= '0.10.0':
        opc.estimate_normals(search_param = open3d.geometry.KDTreeSearchParamHybrid(
            radius = 0.25, max_nn = 30))
    else:
        open3d.geometry.estimate_normals(opc, search_param = open3d.geometry.KDTreeSearchParamHybrid(
                radius = 0.25, max_nn = 30))
    
    open3d.visualization.draw_geometries([opc])
