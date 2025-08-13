from klampt.model.gripperinfo import GripperInfo
from klampt import vis
import sys

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python gripperinfo.py <gripperinfo.json>")
        sys.exit(1)
    gripper = GripperInfo.load(sys.argv[1])
    vis.setWindowTitle("Gripper: " + gripper.name)
    gripper.addToVis()
    vis.loop()