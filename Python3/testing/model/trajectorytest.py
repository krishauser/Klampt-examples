from klampt.model.trajectory import Trajectory,HermiteTrajectory,plot_trajectory_mpl
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

def test_hermite_traj():
    milestones = [[0],[1],[1.5],[0.5]]
    times = [0,1,1.5,2]
    traj = HermiteTrajectory()
    traj.makeSpline(Trajectory(times,milestones))
    print(traj.milestones)
    import matplotlib.pyplot as plt
    ax = plot_trajectory_mpl(traj,dt=0.01)
    ax.axis('equal')
    plt.show()
    
    traj2 = traj.timeWarp(lambda t: t**2)
    ax = plot_trajectory_mpl(traj2,dt=0.01)
    ax.axis('equal')
    plt.show()

    traj2 = traj.timeWarp(lambda t: t**2, 0.1)
    ax = plot_trajectory_mpl(traj2,dt=0.01)
    ax.axis('equal')
    plt.show()

    newtimes = np.linspace(0,traj.times[-1],20)
    traj2 = traj.remesh(newtimes.tolist())[0]
    ax = plot_trajectory_mpl(traj2,dt=None)
    ax.axis('equal')
    plt.show()
    
def test_path_to_traj():
    from klampt.model.trajectory import path_to_trajectory
    import matplotlib.pyplot as plt
    milestones = [[0,0],[1,1],[2,0],[3,1]]
    traj = path_to_trajectory(milestones,dt=0.2)
    ax = plot_trajectory_mpl(traj,dt=0.01)
    ax.axis('equal')
    plt.show()

if __name__ == "__main__":
    #test_traj()
    # test_hermite_traj()
    test_path_to_traj()