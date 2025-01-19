from klampt import Heightmap, Geometry3D
from klampt import vis
from klampt.math import se3,so3
from klampt.vis.colorize import colorize
import cv2
from PIL import Image
import numpy as np

def test_orthographic_heightmap():
    hm = Heightmap()
    # TEST IMAGE HEIGHTMAP
    img_file = '../../../data/terrains/mars_heightmap_data/mars_2020_ctx_dtm_elevation_1024.jpg'
    from PIL import Image
    img = Image.open(img_file)
    img = img.convert('L')
    img = np.asarray(img)
    print("IMAGE SIZE",img.shape,"DTYPE",img.dtype)
    zmax = 0.5
    hm.setHeightImage(img, zmax/255.0)
    hm.setSize(2.0,2.0*img.shape[0]/img.shape[1])
    values = hm.getHeights()    

    print("SIZE",values.shape)
    colorize(hm,value='height',colormap='cool')
    #TEST SEGMENTATION COLORING
    def segmentation_test(event,hm=hm):
        values = hm.heights
        seg = np.zeros(values.shape)
        seg[:,:values.shape[1]//2] = 1
        hm.addProperty('segment',seg)
        colorize(hm,'segment','random')
        geom = Geometry3D(hm)
        vis.add('hm',geom)
        
    #TEST COLOR RETRIEVAL
    def show_color(event,hm=hm):
        cols = hm.getColorImage()
        Image.fromarray(cols).show()
    vis.addAction(segmentation_test,'Segmentation test')
    vis.addAction(show_color,'Show colormap')

    geom = Geometry3D(hm)
    print("Viewport dimensions",hm.viewport.x,hm.viewport.y,hm.viewport.w,hm.viewport.h)
    print("Viewport intrinsics",hm.viewport.fx,hm.viewport.fy,hm.viewport.cx,hm.viewport.cy)
    print("BB",geom.getBB())
    vis.add('origin',se3.identity())
    vis.add('hm',geom)
    # #test conversion to triangle mesh
    # geom_mesh = geom.convert('TriangleMesh')
    # tm = geom_mesh.getTriangleMesh()
    # tm.translate([2.5,0.0,0.0])
    # geom_mesh.setTriangleMesh(tm)
    # vis.add('hm_mesh',geom_mesh,color=(1,0,0,1))
    vis.loop()
    vis.kill()

def test_depthmap():
    hm = Heightmap()
    rgb_file = '../../../data/terrains/sunrgbdv2_data/sunrgbdv2_rgb.jpg'
    depth_file = '../../../data/terrains/sunrgbdv2_data/sunrgbdv2_depth.png'
    extrinsics_file = '../../../data/terrains/sunrgbdv2_data/sunrgbdv2_extrinsics.txt'
    xfov = 1.0166577814521098
    yfov = 0.7916406885147462
    height_range = [0.0,4.0]

    img = cv2.imread(depth_file,cv2.IMREAD_UNCHANGED)  #16-bit depth file
    print("DEPTH IMAGE SIZE",img.shape,"DTYPE",img.dtype,"DEPTH RANGE",np.min(img),np.max(img))
    hm.setHeightImage(img, height_range[1]/(65535.0))
    assert not hm.isPerspective()
    img = Image.open(rgb_file)
    img = np.asarray(img)
    print("RGB IMAGE SIZE",img.shape,"DTYPE",img.dtype)
    hm.setColorImage(img)

    hm.setFOV(xfov,yfov)
    assert hm.isPerspective()

    extrinsics = np.loadtxt(extrinsics_file)
    pose = so3.from_ndarray(extrinsics[:3,:3]),extrinsics[:3,3]
    #add an extra y-z flip
    pose = se3.mul((so3.rotation([1,0,0],-np.pi/2),[0]*3),pose)

    geom = Geometry3D()
    geom.setHeightmap(hm)
    geom.setCurrentTransform(*pose)
    print(pose)
    print("Viewport dimensions",hm.viewport.x,hm.viewport.y,hm.viewport.w,hm.viewport.h)
    print("Viewport intrinsics",hm.viewport.fx,hm.viewport.fy,hm.viewport.cx,hm.viewport.cy)
    print("BB",geom.getBB())
    vis.add('origin',se3.identity())
    vis.add('pose',pose)
    vis.add('hm',geom)
    vis.loop()

if __name__ == '__main__':
    test_orthographic_heightmap()
    test_depthmap()