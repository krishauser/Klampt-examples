from klampt.model.trajectory import Trajectory,SE2Trajectory,SE2HermiteTrajectory
from klampt.model.driving import DifferentiallyFlat2DTrajectory,discretize_differential_drive_poses_se2
import math
import numpy as np
import time

def test_traj():
    times = np.linspace(0, 1, 1000)
    milestones = np.vstack((np.linspace(0,100,1000),np.linspace(0,0,1000))).T.tolist()
    traj = Trajectory(times, milestones)
    t0 = time.time()
    ts = []
    ms = []
    scanIndex = None
    for t in np.arange(0,1,0.0003):
        scanIndex,u = traj.getSegment(t,scanIndex=scanIndex)
        ts.append(t)
        ms.append(traj.evalSegment_state(scanIndex,u))
    duplicate = Trajectory(ts,ms)
    t1 = time.time()
    print("Trajectory eval with scanIndex time:",t1-t0,len(duplicate.milestones))
    t0 = time.time()
    ts = []
    ms = []
    for t in np.arange(0,1,0.0003):
        ts.append(t)
        ms.append(traj.eval(t))
    duplicate = Trajectory(ts,ms)
    t1 = time.time()
    print("Trajectory eval time:",t1-t0,len(duplicate.milestones))
    t0 = time.time()
    duplicate = traj.discretize(0.0003)
    t1 = time.time()
    print("Trajectory discretization time:",t1-t0,len(duplicate.milestones))

def plot_se2_traj(se2_traj):
    import matplotlib.pyplot as plt
    fig,axs = plt.subplots(1,2,figsize=(10,5))
    ax1 = axs[0]
    ax2 = axs[1]
    ax1.scatter([m[0] for m in se2_traj.milestones],[m[1] for m in se2_traj.milestones],label='xy')
    c = [np.cos(m[2]) for m in se2_traj.milestones]
    s = [np.sin(m[2]) for m in se2_traj.milestones]
    ax1.quiver([m[0] for m in se2_traj.milestones],[m[1] for m in se2_traj.milestones],c,s,label='heading')
    ax1.axis('equal')
    ax2.plot(se2_traj.times,[m[0] for m in se2_traj.milestones],label='x')
    ax2.plot(se2_traj.times,[m[1] for m in se2_traj.milestones],label='y')
    ax2.plot(se2_traj.times,[m[2] for m in se2_traj.milestones],label='theta')
    ax2.legend()
    plt.show()

def test_dd_traj():
    traj = DifferentiallyFlat2DTrajectory([0,1,2],[[0,0],[1,0],[1,1]],[[1,0],[0,1],[0,0]])
    se2_traj = traj.discretize_se2(0.1)
    plot_se2_traj(se2_traj)

def test_dd():
    #a = [0,0,-math.pi/5]
    #b = [1,1,math.pi/2]
    #c = [0.8,2,math.pi/2]
    #d = [0.7,2.1,math.pi]
    a = [0,0,0]
    b = [-1,1,math.pi*3/2]
    c = [0,0,0]
    d = [-1,0,0]
    traj = discretize_differential_drive_poses_se2([0,1,2,3],[a,b,c,d],0.02)
    plot_se2_traj(traj)

    b = [0,0,math.pi/2]
    c = [-1,2,math.pi]
    traj = discretize_differential_drive_poses_se2([0,1,2],[a,b,c],0.02)
    plot_se2_traj(traj)

if __name__ == '__main__':
    #test_traj()
    # test_dd_traj()
    test_dd()