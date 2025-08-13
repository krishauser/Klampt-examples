from klampt.model.geometry import merge
from klampt import Geometry3D, Appearance
from klampt.math import so3
from klampt import vis
import numpy as np

a = Geometry3D("../../../data/objects/cube.off")
b = Geometry3D("../../../data/objects/cube.off")
b.transform(so3.identity(),[1.5,0,0])
app1 = a.getAppearance()
face_colors = np.array([[1,0,0,1],[0,1,0,1],[0,0,1,1],[1,1,0,1],[0,1,1,1],[1,0,1,1]],dtype=np.float32)
face_colors = face_colors.repeat(2,axis=0)
app1.setColors(Appearance.FACES,face_colors)
#app1.setColor(1,0,0,1)
a.setAppearance(app1)
app2 = b.getAppearance()
app2.setColor(1,0,0,1)
b.setAppearance(app2)
vis.debug(a,b)
c = merge(a,b)
vis.debug(c)
