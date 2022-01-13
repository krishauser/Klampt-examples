from klampt.model.robotinfo import GripperInfo
import json

robotiq_85 = GripperInfo("robotiq_85",0,[1,2,3,4,5,6,7,8],[0],
    klamptModel="robotiq_85.rob")
robotiq_85.type = "parallel"
robotiq_85.center = (0,0,0.1)
robotiq_85.primaryAxis = (0,0,1)
robotiq_85.secondaryAxis = (1,0,0)
robotiq_85.fingerLength = 0.06
robotiq_85.fingerDepth = 0.01
robotiq_85.fingerWidth = 0.02
robotiq_85.maximumSpan = 0.085 - 0.01
robotiq_85.minimumSpan = 0
robotiq_85.openConfig = [0]*8
robotiq_85.closedConfig = [0.723,0,0.723,-0.723,-0.723,0.723,-0.723,0 ]
#robotiq_85.visualize()

with open('robotiq_85_gripper.json','w') as f:
    json.dump(robotiq_85.toJson(),f)

robotiq_85_ur5 = GripperInfo.mounted(robotiq_85,"ur5_with_robotiq85.rob","gripper:Link_0","robotiq_85-ur5")
robotiq_85_ur5.visualize()

with open('robotiq_85_ur5_gripper.json','w') as f:
    json.dump(robotiq_85_ur5.toJson(),f)

robotiq_140 = GripperInfo("robotiq_140",0,[1,2,4,6,7,9],[0],
    klamptModel="robotiq_140.rob")
robotiq_140.type = "parallel"
robotiq_140.center = (0,0,0.1)
robotiq_140.primaryAxis = (0,0,1)
robotiq_140.secondaryAxis = (0,1,0)
robotiq_140.fingerLength = 0.12
robotiq_140.fingerDepth = 0.01
robotiq_140.fingerWidth = 0.02
robotiq_140.maximumSpan = 0.140 - 0.01
robotiq_140.minimumSpan = 0
robotiq_140.openConfig = [0]*6
robotiq_140.closedConfig = [0.7,0.7,0.7,0.7,0.7,0.7]
#robotiq_140.visualize()

with open('robotiq_140_gripper.json','w') as f:
    json.dump(robotiq_140.toJson(),f)