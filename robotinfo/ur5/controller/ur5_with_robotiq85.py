from klampt.control import RobotInterfaceBase,OmniRobotInterface
from klampt.model.robotinfo import RobotInfo
from ur5_ril import UR5RobotInterface
import os

def make(robotModel, ur5_addr='10.1.1.30', gripper_addr='10.1.1.30'):
    mypath = os.path.split(__file__)[0]
    robotinfofile = os.path.abspath(os.path.join(mypath,'../../robotiq_2finger/robotiq_85_real.json'))
    robotinfo = RobotInfo.load(robotinfofile)
    gripper_controller = robotinfo.controller()
    iface = OmniRobotInterface(robotModel)
    iface.addPhysicalPart('arm',UR5RobotInterface(ur5_addr),[0,1,2,3,4,5])
    iface.addPhysicalPart('gripper',gripper_controller,[6])
    return iface


