from klampt import *

def make_world():
    w = WorldModel()
    w.readFile("../../../data/tx90cupscupboard.xml")
    #w.readFile("../../../data/robots/tx90ball.rob")
    r = w.robot(0)
    q0 = r.getConfig()
    r.randomizeConfig()
    qrand = r.getConfig()
    r.setConfig(q0)
    r.randomizeConfig()
    qrand2 = r.getConfig()
    r.setConfig(q0)

    #show edges of a link
    r.link(0).appearance().setDraw(Appearance.EDGES,True)
    r.link(0).appearance().setColor(Appearance.EDGES,1,1,1,0.2)
    return w


def test_basic_plugin():
    from klampt.vis.glprogram import GLNavigationProgram
    w = make_world()
    vis.add("world",w)

    class KBTest(GLNavigationProgram):
        def __init__(self):
            GLNavigationProgram.__init__(self,'KBTest')
        def keyboardfunc(self,c,x,y):
            print(c,x,y)
    vis.run(KBTest())

if __name__ == '__main__':
    test_basic_plugin()
