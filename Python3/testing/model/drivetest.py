from klampt.model.driving import discretize_differential_drive_poses_se2
import math

if __name__ == '__main__':
    #a = [0,0,-math.pi/5]
    #b = [1,1,math.pi/2]
    #c = [0.8,2,math.pi/2]
    #d = [0.7,2.1,math.pi]
    a = [0,0,0]
    b = [-1,1,math.pi*3/2]
    c = [0,0,0]
    d = [-1,0,0]
    traj = discretize_differential_drive_poses_se2([0,1,2,3],[a,b,c,d],0.02)
    import matplotlib.pyplot as plt
    plt.plot([m[0] for m in traj.milestones],[m[1] for m in traj.milestones],label='xy')
    #plt.plot(traj.times,[m[0] for m in traj.milestones],label='x')
    #plt.plot(traj.times,[m[1] for m in traj.milestones],label='y')
    plt.plot(traj.times,[m[2] for m in traj.milestones],label='theta')
    plt.axis('equal')
    plt.legend()
    plt.show()

    b = [0,0,math.pi/2]
    c = [-1,2,math.pi]
    traj = discretize_differential_drive_poses_se2([0,1,2],[a,b,c],0.02)
    import matplotlib.pyplot as plt
    plt.plot([m[0] for m in traj.milestones],[m[1] for m in traj.milestones],label='y')
    #plt.plot(traj.times,[m[0] for m in traj.milestones],label='x')
    #plt.plot(traj.times,[m[1] for m in traj.milestones],label='y')
    plt.plot(traj.times,[m[2] for m in traj.milestones],label='theta')
    plt.axis('equal')
    plt.legend()
    plt.show()