import klampt
from klampt import vis
from klampt.vis.glinterface import GLPluginInterface
from klampt.vis.glviewport import GLViewport
from klampt.model.collide import WorldCollider

class MouseCapture(GLPluginInterface):
    def __init__(self,world):
        GLPluginInterface.__init__(self)
        self.world = world
        self.q = world.robot(0).getConfig()
        
    def mousefunc(self,button,state,x,y):
        print("Mouse button",button,"state",state,"at point",x,y)
        vp = self.view  # type: GLViewport
        src,dest = vp.click_ray(x,y)
        wc = WorldCollider(world)
        res = wc.rayCast(src,dest)
        if res is not None:
            (obj,pt) = res
            print("  Hit object",obj.getName(),"at point",pt)

    def motionfunc(self,x,y,dx,dy):
        if 'shift' in self.modifiers():
            self.q[2] = float(y)/400
            self.q[3] = float(x)/400
            self.world.robot(0).setConfig(self.q)
            return True
        return False
        
if __name__ == "__main__":
    print("""================================================================
    mouse_capture.py: A simple program where the mouse motion, when
    shift-clicking, gets translated into joint values for an animated robot.
    ========================================================================
    """)

    world = klampt.WorldModel()
    res = world.readFile("../../data/tx90blocks.xml")
    if not res:
        raise RuntimeError("Unable to load world")
    
    plugin = MouseCapture(world)
    vis.add("world",world)
    vis.pushPlugin(plugin)
    vis.setWindowTitle("mouse_capture.py")
    vis.spin(float('inf'))   #shows the window
    vis.kill()


