import numpy as np
import matplotlib.pyplot as plt

velocity = np.genfromtxt("../../../../data/for_edpr_meeting/velocity_derivative_updateMod.csv", delimiter=" ", names=["vx", "vy", "timestamp"])

fig1 = plt.figure()
plt.plot(velocity['timestamp'], velocity['vx'], 'r', linewidth=2)
plt.title('CoM Velocity along x')
plt.xlabel('Time [s]')
plt.ylabel('Vx [pixels/ms]')
plt.show()
