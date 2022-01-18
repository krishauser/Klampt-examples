from klampt.model.calibrate import *
from klampt.model.robotinfo import RobotInfo
from klampt.io import resource
from klampt.math import vectorops,se3,so3
from klampt import *
import sys
import random

world = WorldModel()
if not world.readFile('../../data/robots/ur5.rob'):
    exit(1)
robot = world.robot(0)
camera_obs_link = -1
marker_obs_link = robot.numLinks()-1
calib = RobotExtrinsicCalibration()
calib.robot = robot
calib.cameras['camera'] = CameraInfo(camera_obs_link, {'fx':640,'fy':640,'cx':320,'cy':240})
marker_w = 0.15
calib.markers['marker'] = TransformMarker(marker_obs_link,local_features=[(marker_w/2,marker_w/2,0),
                                                                            (marker_w/2,-marker_w/2,0),
                                                                            (-marker_w/2,-marker_w/2,0),
                                                                            (-marker_w/2,marker_w/2,0)])

#generate synthetic data, corrupted with joint encoder and sensor measurement errors
qmin,qmax = robot.getJointLimits()
numObs = 10
jointEncoderError = 1e-5
sensorErrorRads = 1e-3
sensorErrorMeters = 2e-3
initialGuessPerturbation = 0.1
trueCalibrationConfigs = []
calibrationConfigs = []
trueObservations = []
for obs in range(numObs):
    q0 = [random.uniform(a,b) for (a,b) in zip(qmin,qmax)]
    trueCalibrationConfigs.append(q0)
trueCalibrationConfigs=resource.get("calibration.configs",default=trueCalibrationConfigs,type="Configs",description="Calibration configurations",world=world)
for q0 in trueCalibrationConfigs:
    robot.setConfig(q0)
    dq = [random.uniform(-jointEncoderError,jointEncoderError) for i in range(len(q0))]
    calibrationConfigs.append(vectorops.add(q0,dq))
calib.configurations = trueCalibrationConfigs
calib.frames = [None]*len(trueCalibrationConfigs)
calib.editCalibration()
Tc0 = calib.cameras['camera'].local_coordinates
Tm0 = calib.markers['marker'].local_coordinates
calib.observations = []
#generate template transform observations
for i in range(len(calib.configurations)):
    #TODO: scale error by distance?
    calib.addDetection(se3.identity(),i,'marker',error=[sensorErrorRads]*3 + [sensorErrorMeters]*3)
#generate ground truth
trueObservations = calib.predictedObservations()

#simulate the sensor
calib.configurations = calibrationConfigs
syntheticObservations = calib.predictedObservations(noise=1)
for i in range(len(calib.configurations)):
    calib.observations[i].value = syntheticObservations[i].value

#now need to perturb the intial guess
dr = [random.uniform(-initialGuessPerturbation,initialGuessPerturbation) for i in range(3)]
dx = [random.uniform(-initialGuessPerturbation,initialGuessPerturbation) for i in range(3)]
calib.cameras['camera'].local_coordinates = se3.mul(calib.cameras['camera'].local_coordinates,[so3.from_moment(dr),dx])
dr = [random.uniform(-initialGuessPerturbation,initialGuessPerturbation) for i in range(3)]
dx = [random.uniform(-initialGuessPerturbation,initialGuessPerturbation) for i in range(3)]
calib.markers['marker'].local_coordinates = se3.mul(calib.markers['marker'].local_coordinates,[so3.from_moment(dr),dx])

from klampt.model import coordinates
rgroup = coordinates.addGroup("calibration ground truth")
Tc_link = robot.link(camera_obs_link).getTransform() if camera_obs_link >= 0 else se3.identity()
Tm_link = robot.link(marker_obs_link).getTransform() if marker_obs_link >= 0 else se3.identity()
rgroup.addFrame("camera link",worldCoordinates=Tc_link)
rgroup.addFrame("marker link",worldCoordinates=Tm_link)
rgroup.addFrame("camera (ground truth)",parent="camera link",relativeCoordinates=Tc0)
rgroup.addFrame("marker (ground truth)",parent="marker link",relativeCoordinates=Tm0)
for i,(obs,obs0) in enumerate(zip(calib.observations,trueObservations)):
    rgroup.addFrame("obs"+str(i)+" (ground truth)",parent="camera (ground truth)",relativeCoordinates=obs0.value)
    rgroup.addFrame("obs"+str(i)+" (from camera)",parent="camera (ground truth)",relativeCoordinates=obs.value)
vis.add("simulated coordinates",rgroup)
calib.visualize()

Tc = calib.cameras['camera'].local_coordinates
Tm = calib.markers['marker'].local_coordinates
print ("Initial camera:")
print ("  total error:",vectorops.norm(se3.error(Tc,Tc0)))
print ("  rotation errors:",se3.error(Tc,Tc0)[:3])
print ("  translation errors:",se3.error(Tc,Tc0)[3:])
print ("Initial marker:")
print ("  error:",vectorops.norm(se3.error(Tm,Tm0)))
print ("  rotation errors:",se3.error(Tm,Tm0)[:3])
print ("  translation errors:",se3.error(Tm,Tm0)[3:])
assert calib.cameras['camera'].variable

res = calib.optimize()
print ()
Tc = res[1]['camera']
Tm = res[2]['marker']
print ("Estimated camera transform:",Tc)
print ("  total error:",vectorops.norm(se3.error(Tc,Tc0)))
print ("  rotation errors:",se3.error(Tc,Tc0)[:3])
print ("  translation errors:",se3.error(Tc,Tc0)[3:])
print ("Estimated marker transform:",Tm)
print ("  error:",vectorops.norm(se3.error(Tm,Tm0)))
print ("  rotation errors:",se3.error(Tm,Tm0)[:3])
print ("  translation errors:",se3.error(Tm,Tm0)[3:])
calib.visualize()

vis.kill()

