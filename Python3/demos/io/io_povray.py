import klampt,os
import vapory as vp
from klampt.io import povray,povray_animation
from klampt.vis import GLRealtimeProgram

class GLTest(GLRealtimeProgram):
    def __init__(self,world,sim):
        GLRealtimeProgram.__init__(self,"GLTest")
        self.world = world
        self.sim = sim
        self.anim = False
        
        #povray property
        self.povray_properties = {}
        
        #add 4 lights each with 5 meters from the robot
        povray.add_multiple_lights(self.povray_properties,self.world.robot(0),5.,4,spotlight=True,area=.1)
        
        #tell povray that all terrain will not be modified
        povray.mark_terrain_transient(self.povray_properties,self.world)
        
        #the robot will move, so we remove this line
        #povray.mark_robot_transient(self.povray_properties,self.world.robot(0))
        
        #overwrite material of floor to marble
        fin=vp.Finish('ambient',0.,'diffuse',.5,'specular',.15)
        nor=vp.Normal('granite',0.2,'warp {turbulence 1}','scale',.25)
        povray.set_material(self.povray_properties,self.world.terrain(0),fin,nor)

        #randomize robot link color
        import random
        for l in range(self.world.robot(0).numLinks()):
            lk=self.world.robot(0).link(l)
            lk.appearance().setColor(random.uniform(0.,1.),random.uniform(0.,1.),random.uniform(0.,1.))
    

    def keyboardupfunc(self,c,x,y):
        if c==b'1':
            povray.render_to_file(self.povray_properties,"screenshot.png")
            povray.to_povray(self,self.world,self.povray_properties)
        elif c==b'2':
            if os.path.exists("./animation") and self.anim:
                povray_animation.render_animation("./animation")
            self.anim=not self.anim
        elif c==b'3':
            if os.path.exists("./animation"):
                povray_animation.render_animation("./animation")

    def display_screen(self):
        self.draw_text((20,20),"Press 1 to render screen, 2 to render animation, 3 to compile animation to AVI.")

    def display(self):
        self.sim.updateWorld()
        self.world.drawGL()

    def idle(self):
        if self.anim:
            povray.render_to_animation(self.povray_properties,"./animation")
            povray.to_povray(self,self.world,self.povray_properties)
            
        rfs = sim.controller(0).sensor("RF_ForceSensor")
        print("Sensor values:",rfs.getMeasurements())
        sim.simulate(self.dt)
        return

if __name__ == "__main__":
    print("================================================================")
    print("gl_vis.py: This example demonstrates how to use the GL visualization interface")
    print("   to tie directly into the GUI.")
    print()
    print("   The demo simulates a world and reads a force sensor")
    print("================================================================")
    world = klampt.WorldModel()
    res = world.readFile("../../data/hubo_plane.xml")
    if not res:
        raise RuntimeError("Unable to load world")
    sim = klampt.Simulator(world)
    GLTest(world,sim).run()
