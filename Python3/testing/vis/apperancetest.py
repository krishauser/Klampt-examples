from klampt import WorldModel, Geometry3D
from klampt import vis
from klampt.vis import colorize


def test_per_face_colors():
    #shows bug in visualizer with editor
    w = WorldModel()
    w.readFile("../../../data/robots/baxter.rob")
    r = w.robot(0)
    for i in range(r.numLinks()):
        colorize.colorize(r.link(i),'z','plasma')
    vis.add('world',w)
    vis.edit(('world',r.getName()))
    vis.show()
    vis.spin(float('inf'))


test_per_face_colors()
