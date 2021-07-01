import sys
import os

from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt


filePath_gt = sys.argv[1]
filePath_results = sys.argv[2]

gt = np.genfromtxt(filePath_gt, delimiter=",", names=["x", "y", "timestamp"])
results = np.genfromtxt(filePath_results, delimiter=",", names=["x", "y", "timestamp"])
COM_pix = np.genfromtxt("COM_pix.csv", delimiter=",", names=["v", "timestamp"])
COM_meter = np.genfromtxt("COM_robot.csv", delimiter=",", names=["y", "timestamp"])

fig1 = plt.figure()
plt.plot(gt['timestamp'], gt['y'], 'b', label='ground truth')
plt.plot(results['timestamp'], results['y'], 'r', label='tracker results', linewidth=3)
plt.xlim(5000, 40000)
plt.legend()
plt.title('Tracker Validation')
plt.xlabel('Time [ms]')
plt.ylabel('y CoM [pixels]')
plt.show()


fig2 = plt.figure()
plt.plot(gt['timestamp'], gt['x'], 'b', label='ground truth')
plt.plot(results['timestamp'], results['x'], 'r', label='tracker results', linewidth=3)
plt.xlim(5000, 40000)
plt.legend()
plt.title('Tracker Validation')
plt.xlabel('Time [ms]')
plt.ylabel('x CoM [pixels]')
plt.show()

fig3 = plt.figure()
plt.plot(COM_pix['timestamp'], COM_pix['v'], 'g')
plt.title('y CoM visual space')
plt.xlim([30, 40])
plt.xlabel('Time [ms]')
plt.ylabel('y CoM [pixels]')
plt.show()

fig4 = plt.figure()
plt.plot(COM_meter['timestamp'], COM_meter['y'], 'k')
plt.title('y CoM wrt robot frame')
plt.xlim([30, 40])
plt.xlabel('Time [ms]')
plt.ylabel('y CoM [meters]')
plt.show()

# ax = plt.axes(projection='3d')
# ax.plot3D(x, y, timestamps, 'r')