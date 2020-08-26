from klampt import *
from klampt import vectorops,so3,se3
from klampt import resource
import math
#import matrixops

DO_VISUALIZATION = 1
if DO_VISUALIZATION:
    from klampt import vis
    from klampt.model import coordinates

def point_fit_rotation_3d(apts,bpts):
    """Computes a 3x3 rotation matrix that rotates the points apts to
    minimize the distance to bpts.  Return value is the klampt.so3
    element that minimizes the sum of squared errors ||R*ai-bi||^2."""
    assert len(apts)==len(bpts)

    import numpy as np
    C = np.zeros((3,3))
    for (a,b) in zip(apts,bpts):
	C += np.dot(np.array(a).reshape((3,1)),np.array(b).reshape((1,3)))
    #let A=[a1 ... an]^t, B=[b1 ... bn]^t
    #solve for min sum of squares of E=ARt-B
    #let C=AtB
    #solution is given by CCt = RtCtCR

    #Solve C^tR = R^tC with SVD CC^t = R^tC^tCR
    #CtRX = RtCX
    #C = RCtR
    #Ct = RtCRt
    #=> CCt = RCtCRt
    #solve SVD of C and Ct (giving eigenvectors of CCt and CtC
    #C = UWVt => Ct=VWUt
    #=> UWUt = RVWVtRt
    #=> U=RV => R=UVt
    (U,W,Vt) = np.linalg.svd(C)

    R = np.dot(U,Vt)
    if np.linalg.det(R) < 0:
        #it's a mirror. flip the zero 
        #svd.sortSVs();
        if abs(W[2]) > 1e-2:
            raise RuntimeError("point_fit_rotation_3d: Uhh... what do we do?  SVD of rotation doesn't have a zero singular value")
        #negate the last column of V
        Vt[2,:] *= -1
        R = np.dot(U,Vt)
        assert np.linalg.det(R) > 0
    return R.flatten().tolist()

def point_fit_xform_3d(apts,bpts):
    """Finds a 3D rigid transform that maps the list of points apts to the
    list of points bpts.  Return value is a klampt.se3 element that
    minimizes the sum of squared errors ||T*ai-bi||^2.
    """
    assert len(apts)==len(bpts)
    ca = vectorops.div(reduce(apts,vectorops.add),len(apts))
    cb = vectorops.div(reduce(bpts,vectorops.add),len(bpts))
    arel = [vectorops.sub(a,ca) for a in apts]
    brel = [vectorops.sub(b,cb) for b in bpts]
    R = point_fit_rotation_3d(arel,brel)
    #R minimizes sum_{i=1,...,n} ||R(ai-ca) - (bi-cb)||^2
    t = so3.sub(cb,so3.apply(R,ca))
    return (R,t)

class VectorStats:
    def __init__(self,zero=[0.0],prior=0.0):
        self.average = zero[:]
        #self.variance = [zero[:] for v in zero]
        if not hasattr(prior,'__iter__'):
            self.count = [prior]*len(zero)
        else:
            assert len(zero)==len(prior)
            self.count = prior[:]
    def add(self,value,weight=1.0):
        """Accumulates a new value, with an optional weight factor.  The
        weight can either be a float or a list.  In the latter case it is
        a per-element weight."""
        assert len(value) == len(self.average)
        if not hasattr(weight,'__iter__'):
            weight = [weight]*len(value)
        newcount = vectorops.add(self.count,weight)
        oldweight = [a/b for (a,b) in zip(self.count,newcount)]
        newweight = [1.0/b for b in newcount]
        #oldaverage = self.average
        self.average = [(1.0-w)*a + w*b for (a,b,w) in zip(self.average,value,newweight)]
        #variance = E[vv^T] - E[v]E[v^T]
        #variance(n) = 1/n sum_{1 to n+1} v_i*v_i^T - average(n)*average(n)^T
        #variance(n+1) = 1/(n+1)sum_{1 to n+1} v_i*v_i^T - average(n+1)*average(n+1)^T
        #   = 1/(n+1) sum_{1 to n} v_i*v_i^T + 1/(n+1) v_{n+1}*v_{n+1}^T - average(n+1)*average(n+1)^T
        #   = 1/(n+1)(n variance(n) + average(n)*average(n)^T) - average(n+1)*average(n+1)^T + 1/(n+1) v_{n+1}*v_{n+1}^T
        #   = n/(n+1) variance(n) + 1/(n+1)(v_{n+1}*v_{n+1}^T + average(n)*average(n)^T) - average(n+1)*average(n+1)^T
        #temp1 = matrixops.add(matrixops.outer(oldaverage,oldaverage),matrixops.outer(value,value))
        #temp2 = matrixops.madd(matrixops.mul(oldweight,self.variance),temp1,newweight)
        #self.variance = matrixops.add(temp2,matrixops.outer(self.average,self.average)
        self.count = newcount

class TransformStats:
    def __init__(self,zero=se3.identity(),prior=0.0):
        self.average = zero
        self.count = prior
    def add(self,value,weight=1.0):
        """Accumulates a new value, with an optional weight factor.
        The weight can either be a float or a list.  In the latter
        case, it must be a 6-element vector giving the strength of
        the observation in the Rx,Ry,Rz directions and tx,ty,tz
        directions"""
        assert len(value)==2 and len(value[0])==9 and len(value[1])==3,"Value must be a klampt.se3 type"""
        if hasattr(weight,'__iter__'):
            raise NotImplementedError("Non-uniform transform weights")
        newcount = self.count + weight
        oldweight = self.count / newcount
        newweight = 1.0/newcount
        self.average = se3.interpolate(self.average,value,newweight)
        self.count = newcount

def calibrate_xform_camera(camera_link_transforms,
                           marker_link_transforms,
                           marker_observations,
                           marker_ids,
                           observation_relative_errors=None,
                           camera_initial_guess=None,
                           marker_initial_guess=None,
                           regularizationFactor=0,
                           maxIters=100,
                           tolerance=1e-7):
    """Single camera calibration function for a camera and markers on some
    set of rigid bodies.
    
    Given body transforms and a list of estimated calibration marker observations
    in the camera frame, estimates both the camera transform relative to the
    camera link as well as the marker transforms relative to their links.

    M: is the set of m markers. By default there is at most one marker per link.
       Markers can either be point markers (e.g., a mocap ball), or transform
       markers (e.g., an AR tag or checkerboard pattern).
    O: is the set of n observations, consisting of a reading (Tc_i,Tm_i,o_i,l_i)
       where Tc_i is the camera link's transform, Tm_i is the marker link's
       transform, o_i is the reading which consists of either a point or transform
       estimate in the camera frame, and l_i is the ID of the marker (by default,
       just its link)

    Output: a tuple (err,Tc,marker_dict) where err is the norm of the
    reconstruction residual, Tc is the estimated camera transform relative to the
    camera's link, and marker_dict is a dict mapping each marker id to its
    estimated position or transform on the marker's link.

    Arguments:
    - camera_link: an integer index or a RobotModelLink instance on which
      the camera lies.
    - calibration_configs: a list of the RobotModel configurations q_1,...,q_n
      that generated the marker_observations list.
    - marker_observations: a list of estimated positions or transformations
      of calibration markers o_1,...,o_n, given in the camera's reference
      frame (z forward, x right, y down).

      If o_i is a 3-vector, the marker is considered to be a point marker.
      If a se3 element (R,t) is given, the marker is considered to be
      a transform marker.  You may not mix point and transform observations
      for a single marker ID.
    - marker_ids: a list of marker ID #'s l_1,...,l_n corresponding to
      each observation, e.g., the link index on which each marker lies.
    - observation_relative_errors: if you have an idea of the magnitude of
      each observation error, it can be placed into this list.  Must be
      a list of n floats, 3-lists (point markers), or 6-lists (transform
      markers).  By default, errors will be set proportionally to the
      observed distance between the camera and marker.
    - camera_initial_guess: if not None, an initial guess for the camera transform
    - marker_initial_guess: if not None, a dictionary containing initial guesses
      for the marker transforms
    - regularizationFactor: if nonzero, the optimization penalizes deviation
      of the estimated camera transform and marker transforms from zero
      proportionally to this factor.
    - maxIters: maximum number of iterations for optimization.
    - tolerance: optimization convergence tolerance.  Stops when the change of
      estimates falls below this threshold
    """
    if len(camera_link_transforms) != len(marker_ids):
        raise ValueError("Must provide the same number of marker IDs as camera transforms")
    if len(marker_link_transforms) != len(marker_ids):
        raise ValueError("Must provide the same number of marker IDs as marker transforms")
    if len(marker_observations) != len(marker_ids):
        raise ValueError("Must provide the same number of marker observations as marker transforms")
    #get all unique marker ids
    marker_id_list = list(set(marker_ids))
    #detect marker types
    marker_types = dict((v,None) for v in marker_id_list)
    for i,(obs,id) in enumerate(zip(marker_observations,marker_ids)):
        if len(obs)==3:
            if marker_types[id] == 't':
                raise ValueError("Provided both point and transform observations for observation #%d, id %s\n"%(i,str(id)))
            marker_types[id] = 'p'
        elif len(obs)==2 and len(obs[0])==9 and len(obs[1])==3:
            if marker_types[id] == 'p':
                raise ValueError("Provided both point and transform observations for observation #%d, id %s\n"%(i,str(id)))
            marker_types[id] = 't'
        else:
            raise ValueError("Invalid observation for observation #%d, id %s\n"%(i,str(id)))

    n = len(marker_observations)
    m = len(marker_id_list)

    #get all the observation weights
    observation_weights = []
    if observation_relative_errors is None:
        #default weights: proportional to distance
        for obs in marker_observations:
            if len(obs) == 3:
                observation_weights.append(1.0/vectorops.norm(obs))
            else:
                observation_weights.append(1.0/vectorops.norm(obs[1]))
        observation_weights = [1.0]*len(observation_weights)
    else:
        if len(observation_relative_errors) != n:
            raise ValueError("Invalid length of observation errors")
        for err in observation_relative_errors:
            if hasattr(err,'__iter__'):
                observation_weights.append([1.0/v for v in err])
            else:
                observation_weights.append(1.0/err)

    #initial guesses
    if camera_initial_guess == None:
        camera_initial_guess = se3.identity()
        if any(v == 't' for v in marker_types.itervalues()):
            #estimate camera rotation from point estimates because rotations are more prone to initialization failures
            point_observations = []
            marker_point_rel = []
            for i,obs in enumerate(marker_observations):
                if len(obs)==2:
                    point_observations.append(obs[1])
                else:
                    point_observations.append(obs)
                marker_point_rel.append(se3.mul(se3.inv(camera_link_transforms[i]),marker_link_transforms[i])[1])
            camera_initial_guess = (point_fit_rotation_3d(point_observations,marker_point_rel),[0.0]*3)
            print "Estimated camera rotation from points:",camera_initial_guess
    if marker_initial_guess == None:
        marker_initial_guess = dict((l,(se3.identity() if marker_types[l]=='t' else [0.0]*3)) for l in marker_id_list)
    else:
        marker_initial_guess = marker_initial_guess.copy()
        for l in marker_id_list:
            if l not in marker_initial_guess:
                marker_initial_guess[l] = (se3.identity() if marker_types[l]=='t' else [0.0]*3)
    camera_transform = camera_initial_guess
    marker_transforms = marker_initial_guess.copy()

    if DO_VISUALIZATION:
        rgroup = coordinates.addGroup("calibration")
        rgroup.addFrame("camera link",worldCoordinates=camera_link_transforms[-1])
        rgroup.addFrame("marker link",worldCoordinates=marker_link_transforms[-1])
        rgroup.addFrame("camera estimate",parent="camera link",relativeCoordinates=camera_transform)
        rgroup.addFrame("marker estimate",parent="marker link",relativeCoordinates=marker_transforms.values()[0])
        for i,obs in enumerate(marker_observations):
            rgroup.addFrame("obs"+str(i)+" estimate",parent="camera estimate",relativeCoordinates=obs)
        vis.add("coordinates",rgroup)
        vis.dialog()

    point_observations_only = all(marker_types[marker] == 'p' for marker in marker_id_list)
    xform_observations_only = all(marker_types[marker] == 't' for marker in marker_id_list)
    if not point_observations_only and not xform_observations_only:
        raise NotImplementedError("Can't calibrate camera from mixed point/transform markers yet")
    for iters in range(maxIters):
        #attempt to minimize the error on the following over all observations i
        #camera_link_transform(q_i)*camera_transform*observation_i = marker_link_transform(l_i,q_i)*marker_transform(l_i)
        #first, we'll assume the camera transform is fixed and then optimize the marker transforms.
        #then, we'll assume the marker transforms are fixed and then optimize the camera transform.
        #finally we'll check the error to detect convergence
        #1. Estimate marker transforms from current camera transform
        new_marker_transforms = dict((l,(TransformStats(zero=marker_initial_guess[l],prior=regularizationFactor) if marker_types[l]=='t' else VectorStats(value,zero=[0.0]*3,prior=RegularizationFactor))) for l in marker_id_list)
        for i in xrange(n):
            marker = marker_ids[i]
            Tclink = camera_link_transforms[i]
            Tmlink = marker_link_transforms[i]
            obs = marker_observations[i]
            Trel = se3.mul(se3.inv(Tmlink),se3.mul(Tclink,camera_transform))
            if marker_types[marker] == 't':
                estimate = se3.mul(Trel,obs)
            else:
                estimate = se3.apply(Trel,obs)
            new_marker_transforms[marker].add(estimate,observation_weights[i])
        print "ITERATION",iters
        #print "  ESTIMATED MARKER TRANSFORMS:",dict((k,v.average) for (k,v) in new_marker_transforms.iteritems())
        #2. Estimate camera transform from current marker transforms
        new_camera_transform = TransformStats(zero=camera_initial_guess,prior=regularizationFactor)
        if point_observations_only:
            #TODO: weighted point fitting
            relative_points = []
            for i in xrange(n):
                marker = marker_ids[i]
                Tclink = camera_link_transforms[i]
                Tmlink = marker_link_transforms[i]
                obs = marker_observations[i]
                pRel = se3.apply(se3.inv(Tclink),se3.apply(Tmlink,new_marker_transforms[marker].average))
                relative_points.append(pRel)
            new_camera_transform.add(point_fit_xform_3d(marker_observations,relative_points),sum(observation_weights))
        else:
            for i in xrange(n):
                marker = marker_ids[i]
                Tclink = camera_link_transforms[i]
                Tmlink = marker_link_transforms[i]
                obs = marker_observations[i]
                Trel = se3.mul(se3.inv(Tclink),se3.mul(Tmlink,new_marker_transforms[marker].average))
                estimate = se3.mul(Trel,se3.inv(obs))
                new_camera_transform.add(estimate,observation_weights[i])
        #print "  ESTIMATED CAMERA TRANSFORMS:",new_camera_transform.average
        #3. compute difference between last and current estimates
        diff = 0.0
        diff += vectorops.normSquared(se3.error(camera_transform,new_camera_transform.average))
        for marker in marker_id_list:
            if marker_types[marker]=='t':
                diff += vectorops.normSquared(se3.error(marker_transforms[marker],new_marker_transforms[marker].average))
            else:
                diff += vectorops.distanceSquared(marker_transforms[marker],new_marker_transforms[marker].average)
        camera_transform = new_camera_transform.average
        for marker in marker_id_list:
            marker_transforms[marker] = new_marker_transforms[marker].average
        if math.sqrt(diff) < tolerance:
            #converged!
            print "Converged with diff %g on iteration %d"%(math.sqrt(diff),iters)
            break
        print "  ESTIMATE CHANGE:",math.sqrt(diff)
        error = 0.0
        for i in xrange(n):
            marker = marker_ids[i]
            Tclink = camera_link_transforms[i]
            Tmlink = marker_link_transforms[i]
            obs = marker_observations[i]
            Tc = se3.mul(Tclink,camera_transform)
            if marker_types[marker] == 't':
                Tm = se3.mul(Tmlink,marker_transforms[marker])
                error += vectorops.normSquared(se3.error(se3.mul(Tc,obs),Tm))
            else:
                Tm = se3.apply(Tmlink,marker_transforms[marker])
                error += vectorops.distanceSquared(se3.apply(Tc,obs),Tm)
        print "  OBSERVATION ERROR:",math.sqrt(error)
        #raw_input()
        if DO_VISUALIZATION:
            rgroup.setFrameCoordinates("camera estimate",camera_transform)
            rgroup.setFrameCoordinates("marker estimate",marker_transforms.values()[0])
            for i,obs in enumerate(marker_observations):
                rgroup.setFrameCoordinates("obs"+str(i)+" estimate",obs)
            vis.add("coordinates",rgroup)
            vis.dialog()
    if math.sqrt(diff) >= tolerance:
        print "Max iters reached"
    error = 0.0
    for i in xrange(n):
        marker = marker_ids[i]
        Tclink = camera_link_transforms[i]
        Tmlink = marker_link_transforms[i]
        obs = marker_observations[i]
        Tc = se3.mul(Tclink,camera_transform)
        if marker_types[marker] == 't':
            Tm = se3.mul(Tmlink,marker_transforms[marker])
            error += vectorops.normSquared(se3.error(se3.mul(Tc,obs),Tm))
        else:
            Tm = se3.apply(Tmlink,marker_transforms[marker])
            error += vectorops.distanceSquared(se3.apply(Tc,obs),Tm)
    return (math.sqrt(error),camera_transform,marker_transforms)
    
def calibrate_robot_camera(robot,
                           camera_link,
                           calibration_configs,
                           marker_observations,
                           marker_ids,
                           marker_links=None,
                           observation_relative_errors=None,
                           camera_initial_guess=None,
                           marker_initial_guess=None,
                           regularizationFactor=0,
                           maxIters=100,
                           tolerance=1e-7):
    """Single camera calibration function for a camera and markers on a robot.
    
    Given a robot and a list of estimated calibration marker observations
    in the camera frame, estimates both the camera transform relative to the
    robot's link as well as the marker transforms relative to their links.

    M: is the set of m markers. By default there is at most one marker per link.
       Markers can either be point markers (e.g., a mocap ball), or transform
       markers (e.g., an AR tag or checkerboard pattern).
    O: is the set of n observations, consisting of a reading (q_i,o_i,l_i) where
       q_i is the robot's (sensed) configuration, o_i is the reading which
       consists of either a point or transform estimate in the camera frame,
       and l_i is the ID of the marker (by default, just its link)

    Output: a tuple (err,Tc,marker_dict) where err is the norm of the
    reconstruction residual, Tc is the estimated camera transform relative to the
    camera's link, and marker_dict is a dict mapping each marker id to its
    estimated position or transform on the marker's link.

    Arguments:
    - robot: a RobotModel instance
    - camera_link: an integer index or a RobotModelLink instance on which
      the camera lies.
    - calibration_configs: a list of the RobotModel configurations q_1,...,q_n
      that generated the marker_observations list.
    - marker_observations: a list of estimated positions or transformations
      of calibration markers o_1,...,o_n, given in the camera's reference
      frame (z forward, x right, y down).

      If o_i is a 3-vector, the marker is considered to be a point marker.
      If a se3 element (R,t) is given, the marker is considered to be
      a transform marker.  You may not mix point and transform observations
      for a single marker ID.
    - marker_ids: a list of marker ID #'s l_1,...,l_n corresponding to
      each observation, OR the link indicex on which each marker lies.

      If marker_links is given, there may be more than one marker per link,
      and these entries are marker ID's that index into the marker_links list.
    - marker_links: if provided, this is a dict that maps IDs into link
      indices (or RobotModelLink instances)
    - observation_relative_errors: if you have an idea of the magnitude of
      each observation error, it can be placed into this list.  Must be
      a list of n floats, 3-lists (point markers), or 6-lists (transform
      markers).
    - camera_initial_guess: if not None, an initial guess for the camera transform
    - marker_initial_guess: if not None, a dictionary containing initial guesses
      for the marker transforms
    - regularizationFactor: if nonzero, the optimization penalizes deviation
      of the estimated camera transform and marker transforms from zero
      proportionally to this factor.
    - maxIters: maximum number of iterations for optimization.
    - tolerance: optimization convergence tolerance.  Stops when the change of
      estimates falls below this threshold
    """
    #get the list of all marker IDs, convert all indices to RobotModelLinks
    if len(calibration_configs) != len(marker_observations):
        raise ValueError("Must provide the same number of calibration configs as observations")
    if len(calibration_configs) != len(marker_ids):
        raise ValueError("Must provide the same number of marker IDs as observations")
    if isinstance(camera_link,(int,str)):
        camera_link = robot.link(camera_link)
    marker_id_list = list(set(marker_ids))
    if marker_links is None:
        marker_links = dict((v,robot.link(v)) for v in marker_id_list)
    else:
        for i in marker_id_list:
            if i not in marker_links:
                raise ValueError("There is no marker_link provided for marker id "+str(i))
        marker_links = marker_links.copy()
        for k,v in marker_links.iteritems():
            if isinstance(v,(int,str)):
                marker_links[k] = robot.link(v)
    #get all the transforms for each observation
    camera_link_transforms = []
    marker_link_transforms = []
    for q,m in zip(calibration_configs,marker_ids):
        robot.setConfig(q)
        camera_link_transforms.append(camera_link.getTransform())
        marker_link_transforms.append(marker_links[m].getTransform())
    return calibrate_xform_camera(camera_link_transforms,marker_link_transforms,
                                  marker_observations,marker_ids,
                                  observation_relative_errors=observation_relative_errors,
                                  camera_initial_guess=camera_initial_guess,
                                  marker_initial_guess=marker_initial_guess,
                                  regularizationFactor=regularizationFactor,
                                  maxIters=maxIters,
                                  tolerance=tolerance)

if __name__=="__main__":
    """
    #testing rotation fitting
    apts = [[5,0,0],[0,3,0],[0,0,1]]
    R = so3.rotation((0,0,1),-math.pi/2)
    Rest = point_fit_rotation_3d(apts,[so3.apply(R,a) for a in apts])
    print Rest
    print "Error:"
    for a in apts:
        print vectorops.distance(so3.apply(R,a),so3.apply(Rest,a))
    raw_input()
    """
    
    import sys
    import random
    robot_fn = "../data/robots/baxter_col.rob"
    world = WorldModel()
    if not world.readFile(robot_fn):
        exit(1)
    robot = world.robot(0)
    camera_obs_link = "head_camera"
    marker_obs_link = "left_gripper"
    lc = robot.link(camera_obs_link)
    lm = robot.link(marker_obs_link)
    pc = robot.link(lc.getParent())
    pm = robot.link(lm.getParent())
    Tc0 = se3.mul(se3.inv(pc.getTransform()),lc.getTransform())
    Tm0 = se3.mul(se3.inv(pm.getTransform()),lm.getTransform())
    print "True camera transform",Tc0
    print "True marker transform",Tm0
    print
    camera_link = pc.getName()
    marker_link = pm.getName()

    #generate synthetic data, corrupted with joint encoder and sensor measurement errors
    qmin,qmax = robot.getJointLimits()
    numObs = 10
    jointEncoderError = 1e-5
    sensorErrorRads = 1e-2
    sensorErrorMeters = 2e-3
    trueCalibrationConfigs = []
    calibrationConfigs = []
    trueObservations = []
    observations = []
    for obs in xrange(numObs):
        q0 = [random.uniform(a,b) for (a,b) in zip(qmin,qmax)]
        #don't move head
        for i in range(13):
            q0[i] = 0  
        trueCalibrationConfigs.append(q0)
    trueCalibrationConfigs=resource.get("calibration.configs",default=trueCalibrationConfigs,type="Configs",description="Calibration configurations",world=world)
    for q0 in trueCalibrationConfigs:
        robot.setConfig(q0)
        obs0 = se3.mul(se3.inv(lc.getTransform()),lm.getTransform())
        dq = [random.uniform(-jointEncoderError,jointEncoderError) for i in range(len(q0))]
        dobs = (so3.from_moment([random.uniform(-sensorErrorRads,sensorErrorRads) for i in range(3)]),[random.uniform(-sensorErrorMeters,sensorErrorMeters) for i in range(3)])
        calibrationConfigs.append(vectorops.add(q0,dq))
        observations.append(se3.mul(obs0,dobs))
        trueObservations.append(obs0)

    if DO_VISUALIZATION:    
        rgroup = coordinates.addGroup("calibration ground truth")
        rgroup.addFrame("camera link",worldCoordinates=pc.getTransform())
        rgroup.addFrame("marker link",worldCoordinates=pm.getTransform())
        rgroup.addFrame("camera (ground truth)",parent="camera link",relativeCoordinates=Tc0)
        rgroup.addFrame("marker (ground truth)",parent="marker link",relativeCoordinates=Tm0)
        for i,(obs,obs0) in enumerate(zip(observations,trueObservations)):
            rgroup.addFrame("obs"+str(i)+" (ground truth)",parent="camera (ground truth)",relativeCoordinates=obs0)
            rgroup.addFrame("obs"+str(i)+" (from camera)",parent="camera (ground truth)",relativeCoordinates=obs)
        vis.add("world",world)
        for i,q in enumerate(calibrationConfigs):
            vis.add("config"+str(i),q)
            app = lc.appearance().clone()
            app.setColor(0.5,0.5,0.5,0.1)
            vis.setAppearance("config"+str(i),app)
        vis.add("simulated coordinates",rgroup)
        vis.dialog()

    res = calibrate_robot_camera(robot,camera_link,
                                 calibrationConfigs,
                                 observations,
                                 [marker_link]*len(calibrationConfigs))
    print 
    print "Per-observation reconstruction error:",res[0]/numObs
    print "Estimated camera transform:",res[1]
    print "  total error:",vectorops.norm(se3.error(res[1],Tc0))
    print "  rotation errors:",se3.error(res[1],Tc0)[:3]
    print "  translation errors:",se3.error(res[1],Tc0)[3:]
    print "Estimated marker transform:",res[2][marker_link]
    print "  error:",vectorops.norm(se3.error(res[2][marker_link],Tm0))
    print "  rotation errors:",se3.error(res[2][marker_link],Tm0)[:3]
    print "  translation errors:",se3.error(res[2][marker_link],Tm0)[3:]

    vis.kill()
    
