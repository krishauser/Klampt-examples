import open3d as o3d
from klampt.io import open3d_convert
from klampt import vis, Geometry3D

if __name__ == "__main__":
    print("SIMPLE TEST CONVERTING OPEN3D POINT CLOUDS TO KLAMPT AND BACK")
    opc = o3d.io.read_point_cloud("../../../data/objects/apc/genuine_joe_stir_sticks.pcd")
    print(opc)
    o3d.visualization.draw_geometries([opc])
    kpc = open3d_convert.from_open3d(opc)
    vis.debug(kpc)
    opc2 = open3d_convert.to_open3d(kpc)
    o3d.visualization.draw_geometries([opc2])

    print("SIMPLE TEST CONVERTING OPEN3D VOXEL GRIDS TO KLAMPT AND BACK")
    ogrid = o3d.geometry.VoxelGrid.create_from_point_cloud(opc,voxel_size=0.01)
    kgrid = open3d_convert.from_open3d(ogrid)
    gridgeom = Geometry3D()
    gridgeom.setOccupancyGrid(kgrid)
    vis.debug(gridgeom,kpc)
