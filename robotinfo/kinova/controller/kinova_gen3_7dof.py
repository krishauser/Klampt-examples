import os
from kinova_interface import KinovaGen3RobotInterface

def make(_):
    #HACK: kinova robot interface requires no argument, add a dummy parameter
    """Interface used to refer to robot interfaces by file"""
    from klampt.control.robotinterfaceutils import RobotInterfaceCompleter
    klamptModelFile = os.path.join(os.path.dirname(__file__),
                                   "../../../data/robots/kinova_gen3_7dof.urdf")
    kinova_iface = KinovaGen3RobotInterface(has_gripper=False, klamptModelFile=klamptModelFile)
    return RobotInterfaceCompleter(kinova_iface)