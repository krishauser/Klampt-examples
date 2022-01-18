from kinova_interface import KinovaGen3RobotInterface

def make(_):
    #HACK: kinova robot interface requires no argument, add a dummy parameter
    """Interface used to refer to robot interfaces by file"""
    from klampt.control.robotinterfaceutils import RobotInterfaceCompleter
    return RobotInterfaceCompleter(KinovaGen3RobotInterface())
