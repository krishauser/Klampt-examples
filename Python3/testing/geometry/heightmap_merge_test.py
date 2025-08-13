import klampt
from klampt import vis
from klampt.math import se3, so3
from klampt.math import vectorops as vo
from klampt.model.geometry import upper_heightmap, lower_heightmap, heightmap_normals
import numpy as np

if __name__ == "__main__":
    g = klampt.Geometry3D("../../../data/compound_robots/hrp2/HEAD_LINK1.off")
    g.transform([10,0,0,0,10,0,0,0,10],[0,0,0])
    heights = np.zeros((400,400))
    hm = klampt.Heightmap()
    hm.heights = heights
    hm.setSize(4.0, 4.0)
    uh = upper_heightmap(g, hm)
    uh.setCurrentTransform(uh.getCurrentTransform()[0],[0,0,1])
    vis.debug("upper heightmap", uh, {'color':(1,0,0,1)}, "geom", g, {'color':(1,1,1,0.5)})
    lh = lower_heightmap(g, hm)
    lh.setCurrentTransform(lh.getCurrentTransform()[0],[0,0,-1])
    vis.debug("lower heightmap", lh, {'color':(0,1,0,1)}, "geom", g, {'color':(1,1,1,0.5)})
    
    # vis.add("upper_heightmap", uh, color=(1,0,0,1))
    vis.add("lower_heightmap", lh, color=(0,1,0,1))
    vis.add("geom", g, color=(1,1,1,0.5))
    n = heightmap_normals(lh)
    hm = lh.getHeightmap()
    for i in range(n.shape[0]):
        for j in range(n.shape[1]):
            if i % 10 == 0 and j % 10 == 0:
                if np.linalg.norm(n[i,j]) > 0.01:
                    v = hm.getVertex(i,j)
                    v2 = vo.madd(v, n[i,j], 0.1)
                    v = se3.apply(lh.getCurrentTransform(), v)
                    v2 = se3.apply(lh.getCurrentTransform(), v2)
                    vis.add("normal_%d_%d"%(i,j), [v,v2], color=(0,0,1,1), hide_label=True)
    vis.loop()