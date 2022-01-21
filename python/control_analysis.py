import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

proj = np.genfromtxt("projection_tests.txt", delimiter=" ", names=["y_center", "u", "v_reproj", "v", "u_reproj", "time"])

y_center = proj['y_center'][4497:8078]
time = proj['time'][4497:8078]-proj['time'][4497]

fig1 = plt.figure()
plt.plot(time, y_center, 'r')
plt.title('Projection from the image plane to the robot space' + '\n of the central pixel (152,120), while the robot is moving.', fontsize=14)
plt.xlabel('Time [s]', fontsize=14)
plt.ylabel('y [m]', fontsize=14)
plt.show()
fig1.savefig('center_pixel_projection.jpg',
            format='jpeg',
            dpi=100,
            bbox_inches='tight')

# time = proj['time'][4497:8078]
# u = proj['u'][4497:8078]
# u_reproj = proj['u_reproj'][4497:8078]
# v = proj['v'][4497:8078]
# v_reproj = proj['v_reproj'][4497:8078]
#
# fig2 = plt.figure()
# plt.plot(time, u, 'r', label='xCoM')
# plt.plot(time, u_reproj, 'b', label='projection from robot space to image plane')
# plt.legend()
# plt.title('Comparison of pixel and reprojection to the visual space', fontsize=14)
# plt.xlabel('Time [s]', fontsize=14)
# plt.ylabel('u [pixels]', fontsize=14)
# plt.show()
# fig2.savefig('u_coordinate_reprojection.jpg',
#             format='jpeg',
#             dpi=100,
#             bbox_inches='tight')
#
# fig3 = plt.figure()
# plt.plot(time, v, 'r', label='yCoM')
# plt.plot(time, v_reproj, 'b', label='projection from robot space to image plane')
# plt.legend()
# plt.title('Comparison of pixel and reprojection to the visual space', fontsize=14)
# plt.xlabel('Time [s]', fontsize=14)
# plt.ylabel('v [pixels]', fontsize=14)
# plt.show()
# fig3.savefig('v_coordinate_reprojection.jpg',
#             format='jpeg',
#             dpi=100,
#             bbox_inches='tight')

# info = np.genfromtxt("info.txt", delimiter=" ", names=["y_end_effector", "y_puck", "u_puck", "time"])
#
# y_end_effector = info['y_end_effector'][977:5692]
# y_puck = info['y_puck'][977:5692]
# u_puck = info['u_puck'][977:5692]
# time = info['time'][977:5692]-info['time'][977]
#
# # create figure and axis objects with subplots()
# fig4, ax = plt.subplots()
# # make a plot
# ax.plot(time, y_puck, color="magenta", linewidth=2)
# # set x-axis label
# ax.set_xlabel("time [s]", fontsize=14)
# # set y-axis label
# ax.set_ylabel("y robot space [m]", color="magenta", fontsize=14)
#
# # twin object for two different y-axis on the sample plot
# ax2 = ax.twinx()
# # make a plot with different y-axis using second axis object
# ax2.plot(time, u_puck, color="green", linewidth=2)
# ax2.set_ylabel("u image plane [pixels]", color="green", fontsize=14)
# plt.title("Puck position", fontsize=14)
# plt.show()
# # save the plot as a file
# fig4.savefig('puck_position_image_plane_vs_robot_space_vel_0.8_v>70.jpg',
#             format='jpeg',
#             dpi=100,
#             bbox_inches='tight')
#
# fig5 = plt.figure()
# plt.plot(time, y_end_effector, 'r', label='current y end effector')
# plt.plot(time, y_puck, 'b', label='desired y')
# plt.legend()
# plt.title('Comparison between desired and actual y end effector', fontsize=14)
# plt.xlabel('Time [s]', fontsize=14)
# plt.ylabel('y [m]', fontsize=14)
# plt.show()
# fig5.savefig('puck_vs_end_effector_position_vel_0.8_v>70.jpg',
#             format='jpeg',
#             dpi=100,
#             bbox_inches='tight')
#
# cycleTime = np.genfromtxt("time.txt", delimiter=" ", names=["time"])
# fig6 = plt.figure()
# plt.plot(cycleTime['time'], 'r')
# plt.title('Actual updateModule() loop time ', fontsize=14)
# plt.ylabel('Time [s]', fontsize=14)
# plt.show()
# fig6.savefig('updateModule_period_vel_0.8_v70.jpg',
#             format='jpeg',
#             dpi=100,
#             bbox_inches='tight')