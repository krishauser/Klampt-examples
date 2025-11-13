from impedance import ImpedanceController, ImpedanceControlSettings, AdaptiveImpedanceController, AdaptiveImpedanceControlSettings
from klampt import WorldModel,Simulator
from klampt.math import se3
from klampt.math import vectorops as vo
from klampt.control.simrobotinterface import SimPositionControlInterface
from klampt.control import StepContext
from klampt.io import resource
from klampt import vis
import time
import os

#create world and basic simulator
world = WorldModel()
world.loadFile("impedance_test_data/scooping_setup.xml")
robot = world.robot(0)
q0 = resource.get('start.config', directory='impedance_test_resources', world=world, referenceObject=robot)
robot.setConfig(q0)
sim = Simulator(world)
simcontroller = sim.controller(0)
base_controller = SimPositionControlInterface(simcontroller, sim)

#configure impedance controller
settings = ImpedanceControlSettings(stiffness = 1000, damping=None, mass=20)
settings.tool_center = (0,0.08,0.16)    # a point near the handle
settings.tool_center = (0,0.14,0.26)    # a point near the tip
settings.wrench_sensor = 'filtered_wrench'   #see scooping_bot.xml for how to set up <sensors> tags
settings.feedforward_gain = 0.5
#settings.wrench_sensor = 'wrench'
ft_sensor = simcontroller.sensor(settings.wrench_sensor)
#read wrench sensor frame from model
ee_link = robot.link(robot.driver(robot.numDrivers()-1).getAffectedLink())
ft_sensor_link = simcontroller.sensor('wrench').getLink()
assert simcontroller.sensor('wrench').name != '', "Invalid sensor?"
assert ft_sensor_link.getIndex() >= 0, "Invalid link?"
settings.wrench_sensor_frame = ft_sensor_link.index
settings.wrench_sensor_ee_xform = se3.mul(se3.inv(ee_link.getTransform()),ft_sensor_link.getTransform())

#impedance_controller = ImpedanceController(base_controller, settings)
impedance_controller = AdaptiveImpedanceController(base_controller, settings, AdaptiveImpedanceControlSettings(10,30,20,30))
if not impedance_controller.initialize():
    raise RuntimeError("Could not initialize impedance controller")

vis.add("world",world)
vis.addPlot('impedance')
with StepContext(impedance_controller):
    impedance_controller.setToolCoordinates(settings.tool_center)
    tgt = impedance_controller.get_EE_transform()
    vis.add("target",tgt)
    editor = vis.edit("target")
#uncomment to load a saved view
# if os.path.exists('impedance_test_resources/view.txt'):
#     vp = vis.getViewport()
#     vp.load_file('impedance_test_resources/view.txt')
#     vis.setViewport(vp)

def toggle_impedance():
    if impedance_controller.impedance_active:
        impedance_controller.setControlMode('position')
    else:
        impedance_controller.setControlMode('impedance')
vis.addAction(toggle_impedance,"Toggle impedance","i")

#called every visualization update
def update_cb():
    #below code is required to step the simulation and advance the controller
    with StepContext(impedance_controller):
        if editor.hasFocus():
            target = vis.getItemConfig("target")
            target = (target[:9],target[9:])
            #impedance_controller.setControlMode('impedance')
            impedance_controller.setCartesianPosition(target)
        state = impedance_controller.state()
    #the endStep will perform the impedance control and advance the simulator

    #below code is just for visualization
    #show link frames?
    # for i in range(robot.numDrivers()):
    #     l = robot.driver(i).getAffectedLink()
    #     vis.add(robot.link(i).getName(),robot.link(i).getTransform())
    #show forces and torques
    wrench = ft_sensor.getMeasurements()
    forces = wrench[0:3]
    torques = wrench[3:6]
    force_scale = 0.01
    torque_scale = 0.01
    vis.add("sensed_force",[ft_sensor_link.getTransform()[1],se3.apply(ft_sensor_link.getTransform(),vo.mul(forces,force_scale))],color=(1,0,0,1))
    vis.add("sensed_torque",[ft_sensor_link.getTransform()[1],se3.apply(ft_sensor_link.getTransform(),vo.mul(torques,torque_scale))],color=(1,0.5,0,1))
    wrench = impedance_controller.get_EE_wrench()
    forces = wrench[0:3]
    torques = wrench[3:6]
    tool_pos = vo.add(impedance_controller.get_EE_transform()[1],[0,0,0.])  #add an offset if it makes it easier to see
    vis.add("est_force",[tool_pos,vo.madd(tool_pos,forces,force_scale)],color=(1,0,0,1))
    vis.add("est_torque",[tool_pos,vo.madd(tool_pos,torques,torque_scale)],color=(1,0.5,0,1))
    #show impedance control internals
    if state['T_mass'] is not None:
        vis.add("T_mass",state['T_mass'])
    if state.get('T_attractor',None) is not None:
        vis.add("T_attractor",state['T_attractor'])
    for k,v in state.items():
        if not k.startswith('T_') and k != 'x_dot_mass' and v is not None:
            if isinstance(v,(list,tuple)):
                for i in range(len(v)):
                    if isinstance(v[i],(int,float)):
                        vis.logPlot('impedance',k+'_'+str(i),v[i])
            elif isinstance(v,(int,float)):
                vis.logPlot('impedance',k,v)

    vis.add("qcommanded",robot.configFromDrivers(impedance_controller.commandedPosition()),color=(0,1,0,0.5))
    sim.updateWorld()

#using single-threaded interface to be maximally compatible with Mac and Windows
vis.loop(callback=update_cb)

vis.kill()
