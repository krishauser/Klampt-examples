from klampt import Heightmap, Geometry3D
from klampt import vis
from klampt.math import se3
from klampt.vis.colorize import colorize
import numpy as np

def test_heightmap():
    hm = Heightmap()
    # TEST IMAGE HEIGHTMAP
    img_file = '../../../data/terrains/mars_2020_ctx_dtm_elevation_1024.jpg'
    from PIL import Image
    img = Image.open(img_file)
    img = img.convert('L')
    img = np.asarray(img)
    print("IMAGE SIZE",img.shape,"DTYPE",img.dtype)
    zmax = 0.5
    hm.setHeightImage(img, zmax/255.0)
    hm.setSize(2.0,2.0*img.shape[0]/img.shape[1])
    values = hm.getHeights()

    # TEST SIMPLE HEIGHTMAP
    # hm.resize(4,8)
    # hm.setSize(2.0,1.0)
    # hm.set(0.0)
    # values = hm.getHeights()
    # for i in range(values.shape[1]):
    #     values[values.shape[0]-1,i] = i/float(values.shape[1]-1)
    
    # TEST PERSPECTIVE HEIGHTMAP
    #hm.setFOV(np.radians(45.0),np.radians(90.0))
    hm.setFOV(np.radians(90.0),-1)
    values[:,:] += 0.5

    print("SIZE",values.shape)
    colorize(hm,value='height',colormap='cool')
    #TEST SEGMENTATION COLORING
    # seg = np.zeros(values.shape)
    # seg[:,:values.shape[1]//2] = 1
    # hm.addProperty('segment',seg)
    # colorize(hm,'segment','random')

    geom = Geometry3D()
    geom.setHeightmap(hm)
    print("Viewport dimensions",hm.viewport.x,hm.viewport.y,hm.viewport.w,hm.viewport.h)
    print("Viewport intrinsics",hm.viewport.fx,hm.viewport.fy,hm.viewport.cx,hm.viewport.cy)
    print("BB",geom.getBB())
    geom_mesh = geom.convert('TriangleMesh')
    tm = geom_mesh.getTriangleMesh()
    tm.translate([2.5,0.0,0.0])
    geom_mesh.setTriangleMesh(tm)
    vis.add('origin',se3.identity())
    vis.add('hm',geom)
    vis.add('hm_mesh',geom_mesh,color=(1,0,0,1))
    vis.loop()

if __name__ == '__main__':
    test_heightmap()