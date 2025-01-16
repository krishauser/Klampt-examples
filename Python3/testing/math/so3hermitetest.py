import numpy as np
from klampt.math import so3,vectorops
from klampt.model.trajectory import SO3Trajectory,SO3HermiteTrajectory
from klampt.math.geodesic import SO3Space
import matplotlib.pyplot as plt

times = np.arange(0, 5, 0.1).tolist()
milestones = []
velocities = []
milestones_plot = []
velocities_plot = []
for t in times:
    # rotation around x axis
    milestones.append(so3.from_rotation_vector([np.cos(t), 0, 0]))
    # corresponding angular velocity 
    #velocities.append(so3.mul(so3.cross_product([-np.sin(t), 0, 0]), milestones[-1]))
    velocities.append(so3.cross_product([-np.sin(t), 0, 0]))

    milestones_plot.append(np.cos(t))
    velocities_plot.append(-np.sin(t))


# Create the SO3Trajectory
traj = SO3HermiteTrajectory(times, milestones, velocities)
#traj.discretize(0.05)
#traj.makeSpline(SO3Trajectory(times,milestones))

# re-sample the trajectory at a smaller rate
t = np.arange(0, times[-1], 0.01)
X_ = np.zeros(len(t))
V_ = np.zeros(len(t))

for i in range(len(t)):
    R = traj.eval(t[i])
    dR = traj.deriv(t[i])
    X_[i] = so3.rotation_vector(R)[0]
    #V_[i] = so3.deskew(so3.mul(dR,so3.inv(R)))[0]
    V_[i] = so3.deskew(dR)[0]


plt.plot(times, milestones_plot, "*", label="X")
plt.plot(times, velocities_plot, "*", label="V")

plt.plot(t, X_, label="X_")
plt.plot(t, V_, label="V_")

plt.legend()
plt.show()
