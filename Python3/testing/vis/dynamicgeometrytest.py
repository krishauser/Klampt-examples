from klampt import *
from klampt.io import numpy_convert
import time
import numpy as np


def make_world():
    w = WorldModel()
    w.readFile("../../../data/tx90cupscupboard.xml")
    #w.readFile("../../../data/robots/tx90ball.rob")
    return w

def test_dynamic_point_cloud(duration = float('inf')):
    w = 640
    h = 480
    #w = 320
    #h = 240
    pc = np.zeros((w*h,3))
    colors = np.zeros((w*h,3))
    x = np.linspace(-1,1,w)
    y = np.linspace(-1,1,h)
    for i in range(h):
        pc[i*w:i*w+w,0] = x
    for i in range(w):
        pc[i:h*w:w,1] = y
    colors[:,0] = pc[:,0]*0.5+0.5
    colors[:,1] = 0
    colors[:,2] = pc[:,1]*0.5+0.5
    tstart = time.time()
    t2last = tstart
    tnumpy_avg = 0
    tklampt_avg = 0
    n = 0
    method = 3
    kpc = PointCloud()
    #structured point clouds are a little slower
    # kpc.setSetting("width",str(w))
    # kpc.setSetting("height",str(h))
    # kpc.setSetting("viewpoint",'0 0 10 0 1 0 0 ')  #looking down
    kpc.addProperty("r")
    kpc.addProperty("g")
    kpc.addProperty("b")
    kpc.points = pc
    kpc.properties = colors
    t0 = time.time()
    vis.add('pc',kpc)
    vis.show()
    while vis.shown():
        t0 = time.time()
        pc[:,2] = 0.2*np.sin(2.0*np.dot(pc[:,0:2],[1,1])+(t0-tstart))
        colors[:,1] = (pc[:,2]+0.2)*2.5
        if method==1:
            pc_with_colors = np.hstack((pc,colors))
        t1 = time.time()
        if method == 1:
            kpc = numpy_convert.from_numpy(pc_with_colors,'PointCloud',template=kpc)
        elif method == 2:
            kpc.setPoints(pc)
        elif method == 3: 
            kpc.setPoints(pc)
            kpc.setProperties(colors)
        vis.add('pc',kpc)
        #vis.dirty('pc')
        t2 = time.time()
        tnumpy_avg += 1.0/(n+1)*(t1-t0 - tnumpy_avg)
        tklampt_avg += 1.0/(n+1)*(t2-t1 - tklampt_avg)
        vis.add('nfps','Numpy update: %.2fms'%(1000*tnumpy_avg),position=(10,10),color=(0,0,0,1))
        vis.add('fps','Klampt update: %.2fms'%(1000*tklampt_avg),position=(10,25),color=(0,0,0,1))
        vis.add('drawfps','Overall FPS: %.2f'%(1.0/(t2-t2last)),position=(10,40),color=(0,0,0,1))
        n += 1
        t2last = t2
        time.sleep(0.001)
        if time.time() - tstart > duration:
            vis.show(False)
            break;

def test_dynamic_mesh(duration = float('inf')):
    w = make_world()
    r = w.robot(0)
    res = numpy_convert.to_numpy(r.link(0).geometry(),'Geometry3D')
    T,(verts0,tris) = res
    verts = verts0.copy()
    
    tstart = time.time()
    t2last = tstart
    tnumpy_avg = 0
    tklampt_avg = 0
    n = 0
    method = 1
    kmesh = TriangleMesh()
    #structured point clouds are a little slower
    # kpc.setSetting("width",str(w))
    # kpc.setSetting("height",str(h))
    # kpc.setSetting("viewpoint",'0 0 10 0 1 0 0 ')  #looking down
    kmesh.setVertices(verts0)
    kmesh.setIndices(tris)
    t0 = time.time()
    vis.add('mesh',kmesh)
    vis.show()
    while vis.shown():
        t0 = time.time()
        verts[:,0] = verts0[:,0] + 0.2*np.sin(4.0*verts[:,1]+(t0-tstart))
        t1 = time.time()
        if method == 1:
            kmesh = numpy_convert.from_numpy((verts,tris),'TriangleMesh')
        elif method == 2:
            kmesh.setVertices(verts)
        elif method == 3:
            kmesh.setVertices(verts)
            kmesh.setIndices(tris)
        vis.add('mesh',kmesh)
        #vis.dirty('mesh')
        t2 = time.time()
        tnumpy_avg += 1.0/(n+1)*(t1-t0 - tnumpy_avg)
        tklampt_avg += 1.0/(n+1)*(t2-t1 - tklampt_avg)
        vis.add('nfps','Numpy update: %.2fms'%(1000*tnumpy_avg),position=(10,10),color=(0,0,0,1))
        vis.add('fps','Klampt update: %.2fms'%(1000*tklampt_avg),position=(10,25),color=(0,0,0,1))
        vis.add('drawfps','Overall FPS: %.2f'%(1.0/(t2-t2last)),position=(10,40),color=(0,0,0,1))
        n += 1
        t2last = t2
        time.sleep(0.01)
        if time.time() - tstart > duration:
            vis.show(False)
            break

if __name__ == '__main__':
    test_dynamic_point_cloud(5.0)
    test_dynamic_mesh(5.0)
    vis.kill()
