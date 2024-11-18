import klampt
from klampt import vis
from klampt.vis import GLPluginInterface

class GLTest(GLPluginInterface):
    """Define hooks into the GUI loop to draw and update the simulation"""
    def __init__(self,world,sim):
        GLPluginInterface.__init__(self)
        self.world = world
        self.sim = sim

    def display(self):
        self.sim.updateWorld()
        self.world.drawGL()
        return True

    def idle(self):
        rfs = sim.controller(0).sensor("RF_ForceSensor")
        print("Sensor values:",rfs.getMeasurements())
        sim.simulate(self.dt)
        return True

if __name__ == "__main__":
    print("================================================================")
    print("gl_vis.py: This example demonstrates how to use the GL plugin interface")
    print("   to tie directly into the GUI.")
    print()
    print("   The demo simulates a world and reads a force sensor")
    print()
    print("Note: it is easier to use the vis interface, and we recommend that")
    print("beginners start there")
    print("================================================================")
    world = klampt.WorldModel()
    res = world.readFile("../../data/hubo_plane.xml")
    if not res:
        raise RuntimeError("Unable to load world")
    sim = klampt.Simulator(world)
    print("STARTING vis.run()")
    vis.setWindowTitle("GLPluginInterface Test")
    vis.run(GLTest(world,sim))
    print("END OF vis.run()")
    #equivalent vis code
    # import time
    # vis.add("world",world)
    # vis.show()
    # dt = 1.0/30.0
    # while vis.shown():
    #    t0 = time.time()
    #    sim.simulate(dt)
    #    vis.lock()
    #    sim.updateWorld()
    #    vis.unlock()
    #    rfs = sim.controller(0).sensor("RF_ForceSensor")
    #    print("Sensor values:",rfs.getMeasurements())
    #    t1 = time.time()
    #    time.sleep(max(dt-(t1-t0),0.001))
       