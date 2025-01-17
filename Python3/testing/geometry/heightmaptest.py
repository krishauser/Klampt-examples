from klampt import Heightmap, Geometry3D
from klampt import vis
from klampt.math import se3
from klampt.vis.colorize import colorize
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
    # seg = np.zeros(values.shape)
    # seg[:,:values.shape[1]//2] = 1
    # hm.addProperty('segment',seg)
    # colorize(hm,'segment','random')
    cols = hm.getColors()
    print(cols[:4,:4,:])
    cols = cols.swapaxes(0,1)
    Image.fromarray((cols*255).astype(np.uint8)).show()

    geom = Geometry3D()
    geom.setHeightmap(hm)
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

def test_depthmap():
    hm = Heightmap()
    rgb_file = '../../../data/terrains/sunrgbdv2_data/sunrgbdv2_rgb.jpg'
    depth_file = '../../../data/terrains/sunrgbdv2_data/sunrgbdv2_depth.png'
    xfov = 1.0166577814521098
    yfov = 0.7916406885147462
    height_range = [0.0,4.0]

    from PIL import Image
    img = Image.open(depth_file)
    img = img.convert('L')
    img = np.asarray(img)
    print("DEPTH IMAGE SIZE",img.shape,"DTYPE",img.dtype,"DEPTH RANGE",np.min(img),np.max(img))
    hm.setHeightImage(img, 0.02) #height_range[1]/255.0)
    # img = Image.open(rgb_file)
    # img = np.asarray(img)
    # print("RGB IMAGE SIZE",img.shape,"DTYPE",img.dtype)
    # hm.setColorImage_b3(img)

    values = hm.getHeights()
    hm.setFOV(xfov,yfov)

    # colorize(hm,value='height',colormap='cool')

    geom = Geometry3D()
    geom.setHeightmap(hm)
    print("Viewport dimensions",hm.viewport.x,hm.viewport.y,hm.viewport.w,hm.viewport.h)
    print("Viewport intrinsics",hm.viewport.fx,hm.viewport.fy,hm.viewport.cx,hm.viewport.cy)
    print("BB",geom.getBB())
    vis.add('origin',se3.identity())
    vis.add('hm',geom)
    # geom_mesh = geom.convert('TriangleMesh')
    # tm = geom_mesh.getTriangleMesh()
    # tm.translate([2.5,0.0,0.0])
    # geom_mesh.setTriangleMesh(tm)
    # vis.add('hm_mesh',geom_mesh,color=(1,0,0,1))
    vis.loop()

if __name__ == '__main__':
    #test_orthographic_heightmap()
    test_depthmap()