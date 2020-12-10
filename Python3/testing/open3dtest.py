import open3d as o3d
from klampt.io import open3d_convert
from klampt import vis

if __name__ == "__main__":
    opc = o3d.io.read_point_cloud("../../data/objects/apc/genuine_joe_stir_sticks.pcd")
    print(opc)
    o3d.visualization.draw_geometries([opc])
    kpc = open3d_convert.from_open3d(opc)
    vis.debug(kpc)
    vis.kill()
    opc2 = open3d_convert.to_open3d(kpc)
    o3d.visualization.draw_geometries([opc2])