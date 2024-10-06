import numpy as np
import json
import time
import cv2


def draw_klampt(dir, grasp_id, move_id):
    from klampt import vis
    from klampt import Geometry3D, Appearance, SimRobotSensor
    from klampt.vis.colorize import colorize
    from klampt.model.geometry import point_cloud_normals,point_cloud_colors,point_cloud_set_colors
    from klampt.model import sensing
    from klampt.math import se3

    with open(f'{dir}/camera_color_intrinsics.json','r') as f:
        intrinsics = json.load(f)
    with open(f'{dir}/camera_extrinsic.json','r') as f:
        camera_pose = json.load(f)
    
    # Background image
    color = cv2.imread(f'{dir}/color_grasp_{grasp_id:02d}_move_{move_id:02d}_end.png')
    assert color.shape[1] == intrinsics['width']
    assert color.shape[0] == intrinsics['height']
    #depth = cv2.imread(f'{out_dir}/depth_grasp_{grasp_id:02d}_move_{move_id:02d}_end.png')

    # Arm mesh
    arm_mesh = Geometry3D()
    arm_mesh.loadFile(f'{dir}/grasp_{grasp_id:02d}_move_{move_id:02d}_sim_arm_mesh.ply')

    # Simulated plant point cloud
    sim_plant_pcd = Geometry3D()
    sim_plant_pcd.loadFile(f'{dir}/grasp_{grasp_id:02d}_move_{move_id:02d}_sim_plant.pcd')

    # Generate a colorization of the point cloud
    point_cloud_normals(sim_plant_pcd,estimation_radius=0.005,estimation_viewpoint=(0,-1,1))
    colorize(sim_plant_pcd,'z',lighting=(0,-1,0))
    #colorize(sim_plant_pcd,'z')
    #set transparency
    cols = point_cloud_colors(sim_plant_pcd.getPointCloud(),('r','g','b','a'))
    cols[:,3] = 0.5  
    point_cloud_set_colors(sim_plant_pcd.getPointCloud(),cols,('r','g','b','a'))

    vis.init()
    # set up the extrinsics / intrinsics of the camera -- TODO: klampt can't do non-centered cameras in the Viewport class
    vp = vis.getViewport()
    vp.set_transform(se3.from_homogeneous(camera_pose))
    xfov = np.atan(intrinsics['width']/(intrinsics['fx']*2))*2
    yfov = np.atan(intrinsics['height']/(intrinsics['fy']*2))*2
    vp.fov = np.degrees(xfov)
    vp.w = color.shape[1]
    vp.h = color.shape[0]
    vis.setViewport(vp)
    color = color[:,:,::-1]
    vis.scene().setBackgroundImage(color)
    arm_appearance = Appearance()
    arm_appearance.setColor(1,1,1,0.5)
    arm_appearance.setCreaseAngle(0.5)
    arm_appearance.setSilhouette(0.002)
    vis.add('arm_mesh', arm_mesh, appearance = arm_appearance, hide_label=True)
    plant_appearance = Appearance()
    plant_appearance.setColor(Appearance.VERTICES,1,0,0,0.5)
    plant_appearance.setColors(Appearance.VERTICES,cols.astype(np.float32))
    vis.add('sim_plant_pcd', sim_plant_pcd, appearance = plant_appearance, hide_label=True)

    vis.show()
    iters = 0
    while vis.shown():
        time.sleep(0.01)
        iters += 1
        if iters == 20:  #there's a bug in KrisLibrary that changes the appearance the first time the object is drawn?
            arm_appearance.setColor(1,1,1,0.5)
            arm_appearance.setCreaseAngle(0.5)
            arm_appearance.setSilhouette(0.002)
            plant_appearance.setColor(Appearance.VERTICES,1,0,0,0.5)
            plant_appearance.setColors(Appearance.VERTICES,cols.astype(np.float32))
    vis.kill()

if __name__ == '__main__':
    
    # High visibility action
    #grasp_id, move_id = 4, 7
    # Low visibility action
    grasp_id, move_id = 6, 8

    out_dir = 'overlay_data'

    #draw_o3d(out_dir, grasp_id, move_id)
    draw_klampt(out_dir, grasp_id, move_id)