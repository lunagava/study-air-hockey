import os, numpy as np
from tqdm import tqdm
import matplotlib.pyplot as plt
import matplotlib.transforms as mtrans
from matplotlib import colors
import re
import pandas as pd

def lighten_color(color, amount=0.8):
    """
    Lightens the given color by multiplying (1-luminosity) by the given amount.
    Input can be matplotlib color string, hex string, or RGB tuple.

    Examples:
    >> lighten_color('g', 0.3)
    >> lighten_color('#F034A3', 0.6)
    >> lighten_color((.3,.55,.1), 0.5)
    """
    import matplotlib.colors as mc
    import colorsys
    try:
        c = mc.cnames[color]
    except:
        c = color
    c = colorsys.rgb_to_hls(*mc.to_rgb(c))
    return colorsys.hls_to_rgb(c[0], 1 - amount * (1 - c[1]), c[2])

def calc_distance(p1x, p1y, p2x, p2y):
    return np.sqrt((p2x-p1x)**2+(p2y-p1y)**2)

import matplotlib
from matplotlib.ticker import ScalarFormatter, AutoMinorLocator
params = {'xtick.labelsize': 24,
          'ytick.labelsize': 24,
          'font.size': 13,
          'figure.autolayout': True,  # similar to tight_layout
          'figure.figsize': [6.4, 4.8],  # ratio 1.6 is pleasant
          'axes.titlesize': 13,
          'axes.labelsize': 28,
          'lines.linewidth': 2,
          'lines.markersize': 4,
          'legend.fontsize': 16}

matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
plt.style.use(params)

# d = np.genfromtxt("../../../../data/iros_datasets/exp2/robot_motion/movingCam_exp2.txt", delimiter=" ", names=["t", "x_ee", "y_ee", "z_ee", "gaze_pitch", "gaze_roll", "gaze_yaw", "torso_pitch", "torso_roll", "torso_yaw", "head_x","head_y", "head_z", "head_xa", "head_ya", "head_za", "head_theta"])
#
# y_ee = d["y_ee"]
# time_ee = d["t"]
# ee_displacement = y_ee[1:] - y_ee[:-1]
# time_diff = time_ee[1:] - time_ee[:-1]
#
# vel = ee_displacement/time_diff
# vel[515] = 0.5
#
# plt.figure()
# plt.plot(d['t'], d['gaze_yaw']-d['torso_yaw'], label='Head Yaw', color='#EE82EE')
# plt.plot(d['t'], d['gaze_pitch'],  label='Head Pitch', color='#00BFFF')
# plt.plot(d['t'], d['gaze_roll'], label='Head Roll', color='#00FA9A')
# plt.legend(loc="lower center", mode = "expand", ncol = 3)
# # plt.title('Head joints', fontsize=20)
# plt.xlabel('Time [s]')
# plt.ylabel('Head Joints \n [deg]')
# plt.show()

# plt.figure()
# plt.plot(d['t'], d['head_x'], 'r', label='x')
# plt.plot(d['t'], d['head_y'], 'g', label='y')
# plt.plot(d['t'], d['head_z'], 'b', label='z')
# plt.plot(d['t'], d['head_xa'], 'k', label='xa')
# plt.plot(d['t'], d['head_ya'], 'm', label='ya')
# plt.plot(d['t'], d['head_za'], 'c', label='za')
# plt.plot(d['t'], d['head_theta'], 'y', label='theta')
# plt.legend()
# plt.title('Head reference frame')
# plt.xlabel('Time [s]')
# plt.ylabel('Head frame [m]')
# plt.show()

# plt.figure()
# plt.plot(time_ee[1:], vel, color='#4169E1', alpha=0.8, linewidth=1)
# # plt.title('End effector velocity', fontsize=20)
# plt.xlabel('Time [s]')
# plt.ylabel('End-effector \n Velocity [m/s]')
# plt.show()
#
# plt.figure()
# plt.plot(time_ee, y_ee, color='#6A5ACD')
# # plt.title('End effector velocity', fontsize=20)
# plt.xlabel('Time [s]')
# plt.ylabel('End-effector \n Range of Motion [m]')
# plt.show()

# plt.figure()
# plt.plot(y_ee, d['gaze_yaw']-d['torso_yaw'], color='r', linewidth=1, alpha=0.8)
# plt.legend()
# # plt.title('End effector velocity', fontsize=20)
# plt.xlabel('Head yaw [deg]', fontsize=22)
# plt.xlabel('End-effector Range of Motion [m]', fontsize=22)
# plt.xticks(fontsize=18)
# plt.yticks(fontsize=18)
# plt.show()

# live_test_path = "../../../../data/iros_datasets/live_test"
# live_test_files = sorted([traj for traj in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), live_test_path)) if traj.endswith('.txt')])
# PUCK_latency_list=[]
# PF_latency_list=[]
# CL_latency_list=[]
# for t in tqdm(live_test_files, "Loading live test data"):
#
#     if "puck" in t:
#         PUCK_live_test_traj = np.loadtxt(os.path.join(live_test_path, t), delimiter=" ")[:, :4]
#         PUCK_latency_list.append(PUCK_live_test_traj[:,3])
#     elif "vPFT" in t:
#         PF_live_test_traj = np.loadtxt(os.path.join(live_test_path, t), delimiter=" ")[:, :4]
#         PF_latency_list.append(PF_live_test_traj[:,3])
#     elif "clust" in t:
#         CL_live_test_traj = np.loadtxt(os.path.join(live_test_path, t), delimiter=" ")[:, :4]
#         CL_latency_list.append(CL_live_test_traj[:,3])
#
# PUCK_flat_list = [item for sublist in PUCK_latency_list for item in sublist]
# PF_flat_list = [item for sublist in PF_latency_list for item in sublist]
# CL_flat_list = [item for sublist in CL_latency_list for item in sublist]
#
# PUCK_mean_lat = np.mean(PUCK_flat_list)
# PF_mean_lat = np.mean(PF_flat_list)
# CL_mean_lat = np.mean(CL_flat_list)
#
# labels = ['PUCK', "PF", "Cluster"]
# width = 0.5  # the width of the bars
#
# fig, ax = plt.subplots()
# rects1 = ax.bar(0, PUCK_mean_lat, width,  color='tab:blue')
# rects2 = ax.bar(1, PF_mean_lat, width, color='tab:orange')
# rects3 = ax.bar(2, CL_mean_lat, width, color='tab:green')
#
# # Add some text for labels, title and custom x-axis tick labels, etc.
# ax.set_ylabel('Mean Latency [s]')
# # ax.set_xlabel('\n Algorithm')
# plt.xticks([0, 1, 2], labels, rotation=45)
# plt.yscale("log")
# # plt.ylim([0,np.power(10,-2)])
#
# # ax.set_title('Latency')
# # ax.set_xticks(live_trials, labels
#
# # ax.legend()
#
# ax.bar_label(rects1, padding=3)
# ax.bar_label(rects2, padding=3)
# ax.bar_label(rects3, padding=3)
#
# fig.tight_layout()
#
# plt.show()

exp_number = 1

PUCK_path = "../../../../data/iros_datasets/exp"+str(exp_number)+"/Ours3"
GT_path = "../../../../data/iros_datasets/exp"+str(exp_number)+"/GT3"
PF_path = "../../../../data/iros_datasets/exp"+str(exp_number)+"/vPFT3"
CL_path = "../../../../data/iros_datasets/exp"+str(exp_number)+"/vCluster3"
TOS_path = "../../../../data/iros_datasets/exp"+str(exp_number)+"/TOS"

PUCK_files = sorted([traj for traj in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), PUCK_path)) if traj.endswith('.txt')])
GT_files = sorted([traj for traj in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), GT_path)) if traj.endswith('.txt')])
PF_files = sorted([traj for traj in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), PF_path)) if traj.endswith('.txt')])
CL_files = sorted([traj for traj in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), CL_path)) if traj.endswith('.txt')])
TOS_files = sorted([traj for traj in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), TOS_path)) if traj.endswith('.txt')])

detection = []
PUCK_list = []
GT_list = []
PF_list = []
CL_list = []
TOS_list = []
PUCK_latency = []
PF_latency = []
CL_latency = []
TOS_latency = []

GT_row = 0
GT_column = 0
PUCK_row = 0
PUCK_column = 0
PF_row = 0
PF_column = 0
CL_row = 0
CL_column = 0
TOS_row = 0
TOS_column = 0

fig1, axs1 = plt.subplots(5, 4)
fig2, axs2 = plt.subplots(5, 4)
fig3, axs3 = plt.subplots(5, 4)
# fig4, axs4 = plt.subplots(5, 4)
line_colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k', '#0076a8', '#FF00FF', '#00FFFF', '#FF8C00', '#9370DB', '#8B4513', '#191970', '#FFFF00', '#2F4F4F', '#1E90FF', '#4B0082', '#FF69B4', '#778899']

for t in tqdm(PUCK_files, "Loading PUCK data"):
    PUCK_trajs = np.loadtxt(os.path.join(PUCK_path, t), delimiter=" ")[:, :7]
    PUCK_number = int(re.findall("\d+", t)[0])
    detection.append(PUCK_trajs[0, 0])
    PUCK_trajs = np.delete(PUCK_trajs, (0), axis=0)
    PUCK_list.append(PUCK_trajs)
    PUCK_latency.append(PUCK_trajs[:, 3])
    ROI_number_of_pixels = np.multiply(PUCK_trajs[:, 5],PUCK_trajs[:,6])
    Z = [x for _, x in sorted(zip(ROI_number_of_pixels, PUCK_trajs[:,4]))]
    ROI_number_of_pixels = np.argsort(ROI_number_of_pixels)
    axs1[PUCK_row, PUCK_column].plot(PUCK_trajs[:, 0], PUCK_trajs[:, 1], c=lighten_color(line_colors[PUCK_number - 1]),
                                       label="Seq#" + str(PUCK_number) + " estim", linewidth=1, alpha=0.8)
    axs2[PUCK_row, PUCK_column].plot(PUCK_trajs[:, 0], PUCK_trajs[:, 2], c=lighten_color(line_colors[PUCK_number - 1]),
                                       label="Seq#" + str(PUCK_number) + " estim", linewidth=1, alpha=0.8)
    # axs3[PUCK_row, PUCK_column].plot(PUCK_trajs[:, 0], PUCK_trajs[:, 3], c=lighten_color(line_colors[PUCK_number - 1]),
    #                                  label="Seq#" + str(PUCK_number) + " Ours", linewidth=1, alpha=0.8)
    # axs4[PUCK_row, PUCK_column].plot(ROI_number_of_pixels, PUCK_trajs[:,4], c=lighten_color(line_colors[PUCK_number - 1]),
    #                                  label="Seq#" + str(PUCK_number) + " Ours", linewidth=1, alpha=0.8)
    fig1.suptitle("Static Camera")
    fig2.suptitle("Static Camera")
    PUCK_column += 1
    if PUCK_column % 4 == 0 and PUCK_column != 0:
        PUCK_row += 1
        PUCK_column = 0

for t in tqdm(GT_files, "Loading GROUND TRUTH"):
    GT_trajs = np.loadtxt(os.path.join(GT_path, t), delimiter=" ")[:, :5]
    GT_number = int(re.findall("\d+", t)[0])
    GT_cut = GT_trajs[GT_trajs[:, 0] > detection[GT_number - 1]]
    GT_height = (GT_cut[:, 2] - GT_cut[:, 1])
    GT_ypos = GT_cut[:, 1] + GT_height / 2
    GT_width = (GT_cut[:, 4] - GT_cut[:, 3])
    GT_xpos = GT_cut[:, 3] + GT_width / 2
    concat = np.vstack((GT_cut[:, 0], GT_xpos, GT_ypos))
    GT_list.append(concat.transpose())
    axs1[GT_row, GT_column].plot(GT_cut[:, 0], GT_xpos, c=line_colors[GT_number - 1], label="Seq#" + str(GT_number) + " GT",
                           ls='dashed', linewidth=1.5, alpha=0.8)
    axs2[GT_row, GT_column].plot(GT_cut[:, 0], GT_ypos, c=line_colors[GT_number - 1], label="Seq#" + str(GT_number) + " GT",
                           ls='dashed', linewidth=1.5, alpha=0.8)
    GT_column += 1
    if GT_column % 4 == 0 and GT_column != 0:
        GT_row += 1
        GT_column = 0

for t in tqdm(PF_files, "Loading vPFT data..."):
    PF_trajs = np.loadtxt(os.path.join(PF_path, t), delimiter=" ")[:, :4]
    PF_number = int(re.findall("\d+", t)[0])
    PF_trajs = np.delete(PF_trajs, (0), axis=0)
    PF_list.append(PF_trajs)
    PF_latency.append(PF_trajs[:, 3])
    # axs3[PF_row, PF_column].plot(PF_trajs[:, 0], PF_trajs[:, 3], c=lighten_color(line_colors[PF_number - 1]),
    #                                  label="Seq#" + str(PF_number) + " PF", linewidth=1, alpha=0.8)
    PF_column += 1
    if PF_column % 4 == 0 and PF_column != 0:
        PF_row += 1
        PF_column = 0

for t in tqdm(CL_files, "Loading vCluster trajectories..."):
    CL_trajs = np.loadtxt(os.path.join(CL_path, t), delimiter=" ")[:, :4]
    CL_number = int(re.findall("\d+", t)[0])
    CL_trajs = np.delete(CL_trajs, (0), axis=0)
    CL_list.append(CL_trajs)
    CL_latency.append(CL_trajs[:, 3])
    CL_column += 1
    if CL_column % 4 == 0 and CL_column != 0:
        CL_row += 1
        CL_column = 0

for t in tqdm(TOS_files, "Loading TOS trajectories..."):
    TOS_trajs = np.loadtxt(os.path.join(TOS_path, t), delimiter=" ")[:, :4]
    TOS_number = int(re.findall("\d+", t)[0])
    TOS_trajs = np.delete(TOS_trajs, (0), axis=0)
    TOS_list.append(TOS_trajs)
    TOS_latency.append(TOS_trajs[:, 3])
    TOS_column += 1
    if TOS_column % 4 == 0 and TOS_column != 0:
        TOS_row += 1
        TOS_column = 0


# X and Y Position COMPARISON PUCK vs GT
plt.setp(axs1[-1, :], xlabel='Time [s]')
plt.setp(axs1[:, 0], ylabel='X position [pix]')

plt.setp(axs2[-1, :], xlabel='Time [s]')
plt.setp(axs2[:, 0], ylabel='Y position [pix]')

plt.setp(axs3[-1, :], xlabel='Time [s]')
plt.setp(axs3[:, 0], ylabel='Latency [s]')

lines1 = []
labels1 = []

for axs1 in fig1.axes:
    axLine, axLabel = axs1.get_legend_handles_labels()
    lines1.extend(axLine)
    labels1.extend(axLabel)

fig1.legend(lines1, labels1, loc='center right')

lines2 = []
labels2 = []

for axs2 in fig2.axes:
    axLine, axLabel = axs2.get_legend_handles_labels()
    lines2.extend(axLine)
    labels2.extend(axLabel)

fig2.legend(lines2, labels2, loc='center right')

lines3 = []
labels3 = []

for axs3 in fig3.axes:
    axLine, axLabel = axs3.get_legend_handles_labels()
    lines3.extend(axLine)
    labels3.extend(axLabel)

fig3.legend(lines3, labels3, loc='center right')

# for traj in np.arange(0,9):
#     fig4 = plt.figure()
#     plt.plot(GT_list[traj][:,1], GT_list[traj][:,2], c='tab:red', linewidth=2, ls='dashed', label="GT")
#     plt.axis('off')
#     plt.savefig("/data/iros_datasets/puck_motion_seq"+str(traj)+".png", transparent=True)


# for traj in np.arange(0,9):
#     fig4, axs4 = plt.subplots(3,1)
#     fig4.set_size_inches(10.9, 12.5)
#     axs4[0].plot(PUCK_list[traj][:,1], PUCK_list[traj][:,2], c = 'tab:blue', linewidth = 2, label="PUCK")
#     axs4[0].plot(PF_list[traj][:,1], PF_list[traj][:,2], c = 'tab:orange', linewidth = 1, label="PFT")
#     axs4[0].plot(CL_list[traj][:,1], CL_list[traj][:,2], c = 'tab:green', linewidth = 1, label="Cluster")
#     axs4[0].plot(GT_list[traj][:,1], GT_list[traj][:,2], c = 'tab:red', linewidth = 1, ls='dashed', label="GT")
#     axs4[1].plot(PUCK_list[traj][:,0], PUCK_list[traj][:,1], c = 'tab:blue', linewidth = 2, label="PUCK")
#     axs4[1].plot(PF_list[traj][:,0], PF_list[traj][:,1], c = 'tab:orange', linewidth = 1, label="PFT")
#     axs4[1].plot(CL_list[traj][:,0], CL_list[traj][:,1], c = 'tab:green', linewidth = 1, label="Cluster")
#     axs4[1].plot(GT_list[traj][:,0], GT_list[traj][:,1], c = 'tab:red', linewidth = 1, ls='dashed', label="GT")
#     axs4[2].plot(PUCK_list[traj][:,0], PUCK_list[traj][:,2], c = 'tab:blue', linewidth = 2, label="PUCK")
#     axs4[2].plot(PF_list[traj][:,0], PF_list[traj][:,2], c = 'tab:orange', linewidth = 1, label="PFT")
#     axs4[2].plot(CL_list[traj][:,0], CL_list[traj][:,2], c = 'tab:green', linewidth = 1, label="Cluster")
#     axs4[2].plot(GT_list[traj][:,0], GT_list[traj][:,2], c = 'tab:red', linewidth = 1, ls='dashed', label="GT")
#     axs4[0].legend(bbox_to_anchor=(0, 0.98, 1, 0.2), loc="lower left", mode="expand", borderaxespad=0, ncol=4)
#     axs4[0].set_ylim([0, 380])
#     axs4[1].set_ylim([0, 730])
#     axs4[2].set_ylim([0, 380])
#     plt.setp(axs4[0], ylabel='Y position [pxl]')
#     plt.setp(axs4[0], xlabel='X position [pxl]')
#     plt.setp(axs4[1], ylabel='X position [pxl]')
#     plt.setp(axs4[2], ylabel='Y position [pxl]')
#     plt.setp(axs4[-1], xlabel='Time [s]')

# for traj in np.arange(0,9):
#     fig4, axs4 = plt.subplots(1,2)
#     fig4.set_size_inches(10.9,8.5)
#     axs4[0].plot(PUCK_list[traj][:,0], PUCK_list[traj][:,1], c = 'tab:blue', linewidth = 3, label="PUCK")
#     axs4[0].plot(PF_list[traj][:,0], PF_list[traj][:,1], c = 'tab:orange', linewidth = 2, label="PFT")
#     axs4[0].plot(CL_list[traj][:,0], CL_list[traj][:,1], c = 'tab:green', linewidth = 2, label="Cluster")
#     axs4[0].plot(GT_list[traj][:,0], GT_list[traj][:,1], c = 'tab:red', linewidth = 2, ls='dashed', label="GT")
#     axs4[1].plot(PUCK_list[traj][:,0], PUCK_list[traj][:,2], c = 'tab:blue', linewidth = 3, label="PUCK")
#     axs4[1].plot(PF_list[traj][:,0], PF_list[traj][:,2], c = 'tab:orange', linewidth = 2, label="PFT")
#     axs4[1].plot(CL_list[traj][:,0], CL_list[traj][:,2], c = 'tab:green', linewidth = 2, label="Cluster")
#     axs4[1].plot(GT_list[traj][:,0], GT_list[traj][:,2], c = 'tab:red', linewidth = 2, ls='dashed', label="GT")
#     axs4[0].legend(bbox_to_anchor=(0, 0.98, 1, 0.2), loc="lower left", mode="expand", borderaxespad=0, ncol=4)
#     axs4[0].legend(bbox_to_anchor=(0, 0.98, 1, 0.2), loc="lower left", mode="expand", borderaxespad=0, ncol=4)
#     axs4[0].set_ylim([0, 730])
#     axs4[1].set_ylim([0, 380])
#     plt.setp(axs4[0], ylabel='X position [pxl]')
#     plt.setp(axs4[1], ylabel='Y position [pxl]')
#     # plt.setp(axs4[-1], xlabel='Time [s]')

# traj = 2
# fig4, axs4 = plt.subplots(1,3, figsize=(18,5), gridspec_kw={'width_ratios': [2, 2, 1]})
# axs4[2].plot(PUCK_list[traj][:,0], PUCK_list[traj][:,1], c = 'tab:blue', linewidth = 3, label="PUCK")
# axs4[2].plot(GT_list[traj][:,0], GT_list[traj][:,1], c = 'tab:red', linewidth = 2, ls='dashed', label="GT")
# axs4[0].plot(PUCK_list[traj][:,0], PUCK_list[traj][:,1], c = 'tab:blue', linewidth = 3, label="PUCK")
# axs4[0].plot(PF_list[traj][:,0], PF_list[traj][:,1], c = 'tab:orange', linewidth = 2, label="PFT")
# axs4[0].plot(CL_list[traj][:,0], CL_list[traj][:,1], c = 'tab:green', linewidth = 2, label="Cluster")
# axs4[0].plot(GT_list[traj][:,0], GT_list[traj][:,1], c = 'tab:red', linewidth = 2, ls='dashed', label="GT")
# axs4[1].plot(PUCK_list[traj][:,0], PUCK_list[traj][:,2], c = 'tab:blue', linewidth = 3, label="PUCK")
# axs4[1].plot(PF_list[traj][:,0], PF_list[traj][:,2], c = 'tab:orange', linewidth = 2, label="PFT")
# axs4[1].plot(CL_list[traj][:,0], CL_list[traj][:,2], c = 'tab:green', linewidth = 2, label="Cluster")
# axs4[1].plot(GT_list[traj][:,0], GT_list[traj][:,2], c = 'tab:red', linewidth = 2, ls='dashed', label="GT")
# axs4[0].legend(bbox_to_anchor=(0, 0.98, 1, 0.2), loc="lower left", mode="expand", borderaxespad=0, ncol=4)
# axs4[1].legend(bbox_to_anchor=(0, 0.98, 1, 0.2), loc="lower left", mode="expand", borderaxespad=0, ncol=4)
# axs4[2].legend(bbox_to_anchor=(0, 0.98, 1, 0.2), loc="lower left", mode="expand", borderaxespad=0, ncol=4)
# axs4[0].set_ylim([0, 580])
# axs4[1].set_ylim([0, 350])
# plt.setp(axs4[2], ylabel='Y position [pxl]')
# plt.setp(axs4[0], ylabel='X position [pxl]')
# plt.setp(axs4[1], ylabel='Y position [pxl]')
# plt.setp(axs4[-1], xlabel='Time [s]')
# plt.setp(axs4[-2], xlabel='Time [s]')
# plt.setp(axs4[-3], xlabel='Time [s]')

# plt.figure()
# plt.plot(PUCK_list[traj][:,0],PUCK_latency[traj], 'b')
# plt.xlabel("Time[s]")
# plt.ylabel("Latency[s]")
# plt.show()

time_th = 0.01
PUCK_error_static = []
PF_error_static = []
CL_error_static = []
TOS_error_static = []
PUCK_error_time = []
PF_error_time = []
CL_error_time = []
TOS_error_time = []

# PERCENTAGE OF VALID POSITIONS
track_seq = 0
for gt_seq in GT_list:
    # PUCK_error.clear()
    # error_time.clear()
    for j in range(0, len(gt_seq)):
        PUCK_diff_time = abs(PUCK_list[track_seq][:, 0] - gt_seq[j, 0])
        PUCK_min_time_diff = np.min(PUCK_diff_time)
        PUCK_min_index = np.argmin(PUCK_diff_time)

        PF_diff_time = abs(PF_list[track_seq][:, 0] - gt_seq[j, 0])
        PF_min_time_diff = np.min(PF_diff_time)
        PF_min_index = np.argmin(PF_diff_time)

        CL_diff_time = abs(CL_list[track_seq][:, 0] - gt_seq[j, 0])
        CL_min_time_diff = np.min(CL_diff_time)
        CL_min_index = np.argmin(CL_diff_time)

        TOS_diff_time = abs(TOS_list[track_seq][:, 0] - gt_seq[j, 0])
        TOS_min_time_diff = np.min(TOS_diff_time)
        TOS_min_index = np.argmin(TOS_diff_time)

        if PUCK_min_time_diff < time_th:
            PUCK_error_static.append(calc_distance(PUCK_list[track_seq][PUCK_min_index, 1], PUCK_list[track_seq][PUCK_min_index, 2], gt_seq[j,1], gt_seq[j,2]))
            PUCK_error_time.append(gt_seq[j,0])

        if PF_min_time_diff < time_th:
            PF_error_static.append(
                calc_distance(PF_list[track_seq][PF_min_index, 1], PF_list[track_seq][PF_min_index, 2],
                              gt_seq[j, 1], gt_seq[j, 2]))
            PF_error_time.append(gt_seq[j, 0])

        if CL_min_time_diff < time_th:
            CL_error_static.append(
                calc_distance(CL_list[track_seq][CL_min_index, 1], CL_list[track_seq][CL_min_index, 2],
                              gt_seq[j, 1], gt_seq[j, 2]))
            CL_error_time.append(gt_seq[j, 0])

        if TOS_min_time_diff < time_th:
            TOS_error_static.append(
                calc_distance(TOS_list[track_seq][TOS_min_index, 1], TOS_list[track_seq][TOS_min_index, 2],
                              gt_seq[j, 1], gt_seq[j, 2]))
            TOS_error_time.append(gt_seq[j, 0])

            # error.append(abs(track_list[track_seq][min_index, 1]-gt_seq[j,1]))

    # plt.figure()
    # plt.plot(error_time, PUCK_error)
    # plt.plot(error_time, CL_error)
    # plt.plot(error_time, PF_error)
    # plt.title("Seq #"+str(track_seq))
    # plt.show()
    track_seq = track_seq + 1

np.savetxt("/data/iros_datasets/analysis_percentile/PUCK_static.txt", PUCK_error_static)
np.savetxt("/data/iros_datasets/analysis_percentile/PF_static.txt", PF_error_static)
np.savetxt("/data/iros_datasets/analysis_percentile/CL_static.txt", CL_error_static)
np.savetxt("/data/iros_datasets/analysis_percentile/TOS_static.txt", TOS_error_static)

PUCK_mean_error_static = np.mean(PUCK_error_static)
PUCK_std_error_static = np.std(PUCK_error_static)
PF_mean_error_static = np.mean(PF_error_static)
PF_std_error_static = np.std(PF_error_static)
CL_mean_error_static = np.mean(CL_error_static)
CL_std_error_static = np.std(CL_error_static)
TOS_mean_error_static = np.mean(TOS_error_static)
TOS_std_error_static = np.std(TOS_error_static)
print("PUCK="+str(PUCK_mean_error_static)+"+-"+str(PUCK_std_error_static))
print("PF="+str(PF_mean_error_static)+"+-"+str(PF_std_error_static))
print("CL="+str(CL_mean_error_static)+"+-"+str(CL_std_error_static))
print("TOS="+str(TOS_mean_error_static)+"+-"+str(TOS_std_error_static))

alg_names=["PUCK", "PFT", "Cluster", "TOS"]
color_bar = ['blue', 'skyblue', 'darkcyan']
n_items = np.arange(len(alg_names))
mean_comparison_error = [PUCK_mean_error_static, PF_mean_error_static, CL_mean_error_static, TOS_mean_error_static]
std_comparison_error = [PUCK_std_error_static, PF_std_error_static, CL_std_error_static, TOS_std_error_static]

fig, ax = plt.subplots()
ax.bar(n_items, mean_comparison_error, yerr=std_comparison_error, align='center', ecolor='black', alpha=0.5, capsize=10)
ax.set_ylabel('Error [pix]')
ax.set_xticks(n_items)
ax.set_xticklabels(alg_names)
# ax.set_title('Coefficent of Thermal Expansion (CTE) of Three Metals')
ax.yaxis.grid(True)

# plt.figure()
# plt.plot(GT_list[0][:,0], GT_list[0][:,2], ls="dashed", linewidth=20)
# plt.plot(PUCK_list[0][:,0], PUCK_list[0][:,2])
# plt.show()

PUCK_succ_static = []
PF_succ_static = []
CL_succ_static = []
TOS_succ_static = []
num_thresholds = 30
for i in range(num_thresholds):
    PUCK_succ_static.append(100*(len(np.where(np.array(PUCK_error_static) < i)[0]) / len(PUCK_error_static)))
    PF_succ_static.append(100*(len(np.where(np.array(PF_error_static) < i)[0]) / len(PF_error_static)))
    CL_succ_static.append(100*(len(np.where(np.array(CL_error_static) < i)[0]) / len(CL_error_static)))
    TOS_succ_static.append(100*(len(np.where(np.array(TOS_error_static) < i)[0]) / len(TOS_error_static)))

#
# # MEAN LATENCY
# PUCK_latency_global = [item for sublist in PUCK_latency for item in sublist]
# PF_latency_global = [item for sublist in PF_latency for item in sublist]
# CL_latency_global = [item for sublist in CL_latency for item in sublist]
#
# PUCK_mean_latency = np.mean(PUCK_latency_global)
# PUCK_std_error_latency = np.std(PUCK_latency_global)
# PF_mean_latency = np.mean(PF_latency_global)
# PF_std_error_latency = np.std(PF_latency_global)
# CL_mean_latency = np.mean(CL_latency_global)
# CL_std_error_latency = np.std(CL_latency_global)
#
# comparison_mean_latency = [PUCK_mean_latency, PF_mean_latency, CL_mean_latency]
# comparison_std_latency = [PUCK_std_error_latency, PF_std_error_latency, CL_std_error_latency]
#
# fig, ax = plt.subplots()
# ax.bar(n_items, comparison_mean_latency, yerr=comparison_std_latency, align='center', ecolor='red', alpha=0.5, capsize=10)
# ax.set_ylabel('Latency [s]')
# ax.set_xticks(n_items)
# ax.set_xticklabels(alg_names)
# # ax.set_title('Coefficent of Thermal Expansion (CTE) of Three Metals')
# ax.yaxis.grid(True)

# plt.figure()
# plt.plot(gt_list[2][:,0], gt_list[2][:,2], 'k', ls='dashed', linewidth=5, label="GT")
# plt.plot(track_list[2][:,0], track_list[2][:,2], 'r', label="TRACKER")
# plt.legend()
# plt.title("Comparison with the ground truth")
# plt.xlabel("Time [s]")
# plt.ylabel("Y Position [pix]")
# plt.show()
#
# plt.figure()
# plt.plot(gt_list[2][:,0], gt_list[2][:,1], 'k', ls='dashed', linewidth=5, label="GT")
# plt.plot(track_list[2][:,0], track_list[2][:,1], 'r', label="TRACKER")
# plt.legend()
# plt.title("Comparison with the ground truth")
# plt.xlabel("Time [s]")
# plt.ylabel("X Position [pix]")
# plt.show()
#
# plt.figure()
# plt.plot(error_time, error)
# plt.title("Error over time")
# plt.xlabel("Time [s]")
# plt.ylabel("Error [pix]")
# plt.show()
# UPDATE RATE OVER TIME
# plt.figure()
# plt.plot(traj_exp1[:, 0], traj_exp1[:, 3])
# plt.xlabel('Time [ms]')
# plt.ylabel('Frequency [Hz]')
# plt.show()

# x_axis_kernel = np.arange(0,14)*0.1+0
# kernel_value = -0.2*np.arange(0,14)
# for counter in range(len(x_axis_kernel)):
#     if x_axis_kernel[counter]>0.4 and x_axis_kernel[counter]<=0.9:
#         kernel_value[counter] = 1
#     elif x_axis_kernel[counter]>0.9:
#         kernel_value[counter] = -0.2
#     else:
#         kernel_value[counter]= 0
#
# fig = plt.figure()
# fig.set_size_inches(8.9, 7)
# plt.step(x_axis_kernel, kernel_value, 'k', linewidth=4)
# plt.title("Kernel Matrix Filling",fontsize=30)
# plt.xticks(fontsize=20)
# plt.yticks(fontsize=20)
# plt.xlabel("Ellipse Equation Value: "+r"$\frac{(x-xC)^2}{a^2}+\frac{(y-yC)^2}{b^2}$"+"\n", fontsize=30)
# plt.ylabel("Kernel Element Value", fontsize=30)
# plt.show()

exp_number = 2

PUCK_path = "../../../../data/iros_datasets/exp"+str(exp_number)+"/Ours"
GT_path = "../../../../data/iros_datasets/exp"+str(exp_number)+"/GT"
PF_path = "../../../../data/iros_datasets/exp"+str(exp_number)+"/vPFT"
CL_path = "../../../../data/iros_datasets/exp"+str(exp_number)+"/vCluster"
# PIM_path = "../../../../data/iros_datasets/exp"+str(exp_number)+"/PIM"
TOS_path = "../../../../data/iros_datasets/exp"+str(exp_number)+"/TOS"

PUCK_files = sorted([traj for traj in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), PUCK_path)) if traj.endswith('.txt')])
GT_files = sorted([traj for traj in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), GT_path)) if traj.endswith('.txt')])
PF_files = sorted([traj for traj in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), PF_path)) if traj.endswith('.txt')])
CL_files = sorted([traj for traj in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), CL_path)) if traj.endswith('.txt')])

detection = []
PUCK_list = []
GT_list = []
PF_list = []
CL_list = []
PIM_list = []
TOS_list = []
PUCK_latency = []
PF_latency = []
CL_latency = []
TOS_latency = []
PIM_latency = []

GT_row = 0
GT_column = 0
PUCK_row = 0
PUCK_column = 0
PF_row = 0
PF_column = 0
CL_row = 0
CL_column = 0
PIM_row = 0
PIM_column = 0
TOS_row = 0
TOS_column = 0

fig1, axs1 = plt.subplots(5, 4)
fig2, axs2 = plt.subplots(5, 4)
fig3, axs3 = plt.subplots(5, 4)
# fig4, axs4 = plt.subplots(5, 4)
line_colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k', '#0076a8', '#FF00FF', '#00FFFF', '#FF8C00', '#9370DB', '#8B4513', '#191970', '#FFFF00', '#2F4F4F', '#1E90FF', '#4B0082', '#FF69B4', '#778899']

for t in tqdm(PUCK_files, "Loading PUCK data"):
    PUCK_trajs = np.loadtxt(os.path.join(PUCK_path, t), delimiter=" ")[:, :7]
    PUCK_number = int(re.findall("\d+", t)[0])
    detection.append(PUCK_trajs[0, 0])
    PUCK_trajs = np.delete(PUCK_trajs, (0), axis=0)
    PUCK_list.append(PUCK_trajs)
    PUCK_latency.append(PUCK_trajs[:, 3])
    ROI_number_of_pixels = np.multiply(PUCK_trajs[:, 5],PUCK_trajs[:,6])
    Z = [x for _, x in sorted(zip(ROI_number_of_pixels, PUCK_trajs[:,4]))]
    ROI_number_of_pixels = np.argsort(ROI_number_of_pixels)
    axs1[PUCK_row, PUCK_column].plot(PUCK_trajs[:, 0], PUCK_trajs[:, 1], c=lighten_color(line_colors[PUCK_number - 1]),
                                       label="Seq#" + str(PUCK_number) + " estim", linewidth=1, alpha=0.8)
    axs2[PUCK_row, PUCK_column].plot(PUCK_trajs[:, 0], PUCK_trajs[:, 2], c=lighten_color(line_colors[PUCK_number - 1]),
                                       label="Seq#" + str(PUCK_number) + " estim", linewidth=1, alpha=0.8)
    # axs3[PUCK_row, PUCK_column].plot(PUCK_trajs[:, 0], PUCK_trajs[:, 3], c=lighten_color(line_colors[PUCK_number - 1]),
    #                                  label="Seq#" + str(PUCK_number) + " Ours", linewidth=1, alpha=0.8)
    # axs4[PUCK_row, PUCK_column].plot(ROI_number_of_pixels, PUCK_trajs[:,4], c=lighten_color(line_colors[PUCK_number - 1]),
    #                                  label="Seq#" + str(PUCK_number) + " Ours", linewidth=1, alpha=0.8)
    fig1.suptitle("Static Camera")
    fig2.suptitle("Static Camera")
    PUCK_column += 1
    if PUCK_column % 4 == 0 and PUCK_column != 0:
        PUCK_row += 1
        PUCK_column = 0

for t in tqdm(GT_files, "Loading GROUND TRUTH"):
    GT_trajs = np.loadtxt(os.path.join(GT_path, t), delimiter=" ")[:, :5]
    GT_number = int(re.findall("\d+", t)[0])
    GT_cut = GT_trajs[GT_trajs[:, 0] > detection[GT_number - 1]]
    GT_height = (GT_cut[:, 2] - GT_cut[:, 1])
    GT_ypos = GT_cut[:, 1] + GT_height / 2
    GT_width = (GT_cut[:, 4] - GT_cut[:, 3])
    GT_xpos = GT_cut[:, 3] + GT_width / 2
    concat = np.vstack((GT_cut[:, 0], GT_xpos, GT_ypos))
    GT_list.append(concat.transpose())
    axs1[GT_row, GT_column].plot(GT_cut[:, 0], GT_xpos, c=line_colors[GT_number - 1], label="Seq#" + str(GT_number) + " GT",
                           ls='dashed', linewidth=1.5, alpha=0.8)
    axs2[GT_row, GT_column].plot(GT_cut[:, 0], GT_ypos, c=line_colors[GT_number - 1], label="Seq#" + str(GT_number) + " GT",
                           ls='dashed', linewidth=1.5, alpha=0.8)
    GT_column += 1
    if GT_column % 4 == 0 and GT_column != 0:
        GT_row += 1
        GT_column = 0

for t in tqdm(PF_files, "Loading vPFT data..."):
    PF_trajs = np.loadtxt(os.path.join(PF_path, t), delimiter=" ")[:, :4]
    PF_number = int(re.findall("\d+", t)[0])
    PF_trajs = np.delete(PF_trajs, (0), axis=0)
    PF_list.append(PF_trajs)
    PF_latency.append(PF_trajs[:, 3])
    # axs3[PF_row, PF_column].plot(PF_trajs[:, 0], PF_trajs[:, 3], c=lighten_color(line_colors[PF_number - 1]),
    #                                  label="Seq#" + str(PF_number) + " PF", linewidth=1, alpha=0.8)
    PF_column += 1
    if PF_column % 4 == 0 and PF_column != 0:
        PF_row += 1
        PF_column = 0

for t in tqdm(CL_files, "Loading vCluster trajectories..."):
    CL_trajs = np.loadtxt(os.path.join(CL_path, t), delimiter=" ")[:, :4]
    CL_number = int(re.findall("\d+", t)[0])
    CL_trajs = np.delete(CL_trajs, (0), axis=0)
    CL_list.append(CL_trajs)
    CL_latency.append(CL_trajs[:, 3])
    CL_column += 1
    if CL_column % 4 == 0 and CL_column != 0:
        CL_row += 1
        CL_column = 0

for t in tqdm(CL_files, "Loading vTOS trajectories..."):
    TOS_trajs = np.loadtxt(os.path.join(TOS_path, t), delimiter=" ")[:, :4]
    TOS_number = int(re.findall("\d+", t)[0])
    TOS_trajs = np.delete(TOS_trajs, (0), axis=0)
    TOS_list.append(TOS_trajs)
    TOS_latency.append(TOS_trajs[:, 3])
    TOS_column += 1
    if TOS_column % 4 == 0 and TOS_column != 0:
        TOS_row += 1
        TOS_column = 0

# for t in tqdm(CL_files, "Loading vPIM trajectories..."):
#     PIM_trajs = np.loadtxt(os.path.join(PIM_path, t), delimiter=" ")[:, :4]
#     PIM_number = int(re.findall("\d+", t)[0])
#     PIM_trajs = np.delete(PIM_trajs, (0), axis=0)
#     PIM_list.append(PIM_trajs)
#     PIM_latency.append(PIM_trajs[:, 3])
#     PIM_column += 1
#     if PIM_column % 4 == 0 and PIM_column != 0:
#         PIM_row += 1
#         PIM_column = 0

# X and Y Position COMPARISON PUCK vs GT
plt.setp(axs1[-1, :], xlabel='Time [s]')
plt.setp(axs1[:, 0], ylabel='X position [pix]')

plt.setp(axs2[-1, :], xlabel='Time [s]')
plt.setp(axs2[:, 0], ylabel='Y position [pix]')

# plt.setp(axs3[-1, :], xlabel='Time [s]')
# plt.setp(axs3[:, 0], ylabel='Latency [s]')

# for traj in np.arange(0,9):
#     fig4 = plt.figure()
#     plt.plot(GT_list[traj][:,1], GT_list[traj][:,2], c='tab:red', linewidth=3, label="GT")
#     plt.axis('off')
#     plt.savefig("/data/iros_datasets/puck_motion_moving_seq"+str(traj)+".png", transparent=True)

lines1 = []
labels1 = []

for axs1 in fig1.axes:
    axLine, axLabel = axs1.get_legend_handles_labels()
    lines1.extend(axLine)
    labels1.extend(axLabel)

fig1.legend(lines1, labels1, loc='center right')

lines2 = []
labels2 = []

for axs2 in fig2.axes:
    axLine, axLabel = axs2.get_legend_handles_labels()
    lines2.extend(axLine)
    labels2.extend(axLabel)

fig2.legend(lines2, labels2, loc='center right')

lines3 = []
labels3 = []

# for axs3 in fig3.axes:
#     axLine, axLabel = axs3.get_legend_handles_labels()
#     lines3.extend(axLine)
#     labels3.extend(axLabel)
#
# fig3.legend(lines3, labels3, loc='center right')

# for traj in np.arange(0,9):
#     fig4, axs4 = plt.subplots(3,1)
#     fig4.set_size_inches(10.9, 12.5)
#     axs4[0].plot(PUCK_list[traj][:,1], PUCK_list[traj][:,2], c = 'tab:blue', linewidth = 2, label="PUCK")
#     axs4[0].plot(PF_list[traj][:,1], PF_list[traj][:,2], c = 'tab:orange', linewidth = 1, label="PFT")
#     axs4[0].plot(CL_list[traj][:,1], CL_list[traj][:,2], c = 'tab:green', linewidth = 1, label="Cluster")
#     axs4[0].plot(GT_list[traj][:,1], GT_list[traj][:,2], c = 'tab:red', linewidth = 1, ls='dashed', label="GT")
#     axs4[0].set_ylim(max(PUCK_list[traj][:,2]), min(CL_list[traj][:,2])-10)
#     axs4[1].plot(PUCK_list[traj][:,0], PUCK_list[traj][:,1], c = 'tab:blue', linewidth = 3, label="PUCK")
#     axs4[1].plot(PF_list[traj][:,0], PF_list[traj][:,1], c = 'tab:orange', linewidth = 2, label="PFT")
#     axs4[1].plot(CL_list[traj][:,0], CL_list[traj][:,1], c = 'tab:green', linewidth = 2, label="Cluster")
#     axs4[1].plot(GT_list[traj][:,0], GT_list[traj][:,1], c = 'tab:red', linewidth = 2, ls='dashed', label="GT")
#     axs4[2].plot(PUCK_list[traj][:,0], PUCK_list[traj][:,2], c = 'tab:blue', linewidth = 3, label="PUCK")
#     axs4[2].plot(PF_list[traj][:,0], PF_list[traj][:,2], c = 'tab:orange', linewidth = 2, label="PFT")
#     axs4[2].plot(CL_list[traj][:,0], CL_list[traj][:,2], c = 'tab:green', linewidth = 2, label="Cluster")
#     axs4[2].plot(GT_list[traj][:,0], GT_list[traj][:,2], c = 'tab:red', linewidth = 2, ls='dashed', label="GT")
#     axs4[0].legend(bbox_to_anchor=(0, 0.98, 1, 0.2), loc="lower left", mode="expand", borderaxespad=0, ncol=4)
#     axs4[1].set_ylim([0, 730])
#     axs4[2].set_ylim([0, 380])
#     plt.setp(axs4[0], ylabel='Y position [pxl]')
#     plt.setp(axs4[0], xlabel='X position [pxl]')
#     plt.setp(axs4[1], ylabel='X position [pxl]')
#     plt.setp(axs4[2], ylabel='Y position [pxl]')
#     plt.setp(axs4[-1], xlabel='Time [s]')

# for traj in np.arange(0,9):
#     fig4, axs4 = plt.subplots(1,2)
#     fig4.set_size_inches(10.9, 8.5)
#     axs4[0].plot(PUCK_list[traj][:,0], PUCK_list[traj][:,1], c = 'tab:blue', linewidth = 2, label="PUCK")
#     axs4[0].plot(PF_list[traj][:,0], PF_list[traj][:,1], c = 'tab:orange', linewidth = 1, label="PFT")
#     axs4[0].plot(CL_list[traj][:,0], CL_list[traj][:,1], c = 'tab:green', linewidth = 1, label="Cluster")
#     axs4[0].plot(GT_list[traj][:,0], GT_list[traj][:,1], c = 'tab:red', linewidth = 1, ls='dashed', label="GT")
#     axs4[1].plot(PUCK_list[traj][:,0], PUCK_list[traj][:,2], c = 'tab:blue', linewidth = 2, label="PUCK")
#     axs4[1].plot(PF_list[traj][:,0], PF_list[traj][:,2], c = 'tab:orange', linewidth = 1, label="PFT")
#     axs4[1].plot(CL_list[traj][:,0], CL_list[traj][:,2], c = 'tab:green', linewidth = 1, label="Cluster")
#     axs4[1].plot(GT_list[traj][:,0], GT_list[traj][:,2], c = 'tab:red', linewidth = 1, ls='dashed', label="GT")
#     axs4[0].legend(bbox_to_anchor=(0, 0.98, 1, 0.2), loc="lower left", mode="expand", borderaxespad=0, ncol=4)
#     axs4[0].set_ylim([0, 730])
#     axs4[1].set_ylim([0, 380])
#     plt.setp(axs4[0], ylabel='X position [pxl]')
#     plt.setp(axs4[1], ylabel='Y position [pxl]')
#     plt.setp(axs4[-1], xlabel='Time [s]')

time_th = 0.01
PUCK_error_moving = []
PF_error_moving = []
CL_error_moving = []
TOS_error_moving = []
PUCK_error_time = []
PF_error_time = []
CL_error_time = []

# PERCENTAGE OF VALID POSITIONS
track_seq = 0
for gt_seq in GT_list:
    # PUCK_error.clear()
    # error_time.clear()
    for j in range(0, len(gt_seq)):
        PUCK_diff_time = abs(PUCK_list[track_seq][:, 0] - gt_seq[j, 0])
        PUCK_min_time_diff = np.min(PUCK_diff_time)
        PUCK_min_index = np.argmin(PUCK_diff_time)

        PF_diff_time = abs(PF_list[track_seq][:, 0] - gt_seq[j, 0])
        PF_min_time_diff = np.min(PF_diff_time)
        PF_min_index = np.argmin(PF_diff_time)

        CL_diff_time = abs(CL_list[track_seq][:, 0] - gt_seq[j, 0])
        CL_min_time_diff = np.min(CL_diff_time)
        CL_min_index = np.argmin(CL_diff_time)

        TOS_diff_time = abs(TOS_list[track_seq][:, 0] - gt_seq[j, 0])
        TOS_min_time_diff = np.min(TOS_diff_time)
        TOS_min_index = np.argmin(TOS_diff_time)

        if PUCK_min_time_diff < time_th:
            PUCK_error_moving.append(calc_distance(PUCK_list[track_seq][PUCK_min_index, 1], PUCK_list[track_seq][PUCK_min_index, 2], gt_seq[j,1], gt_seq[j,2]))
            PUCK_error_time.append(gt_seq[j,0])

        if PF_min_time_diff < time_th:
            PF_error_moving.append(
                calc_distance(PF_list[track_seq][PF_min_index, 1], PF_list[track_seq][PF_min_index, 2],
                              gt_seq[j, 1], gt_seq[j, 2]))
            PF_error_time.append(gt_seq[j, 0])

        if CL_min_time_diff < time_th:
            CL_error_moving.append(
                calc_distance(CL_list[track_seq][CL_min_index, 1], CL_list[track_seq][CL_min_index, 2],
                              gt_seq[j, 1], gt_seq[j, 2]))
            CL_error_time.append(gt_seq[j, 0])

        # if PIM_min_time_diff < time_th:
        #     PIM_error_moving.append(
        #         calc_distance(PIM_list[track_seq][CL_min_index, 1], PIM_list[track_seq][PIM_min_index, 2],
        #                       gt_seq[j, 1], gt_seq[j, 2]))
        #     PIM_error_time.append(gt_seq[j, 0])

        if TOS_min_time_diff < time_th:
            TOS_error_moving.append(
                calc_distance(TOS_list[track_seq][CL_min_index, 1], TOS_list[track_seq][TOS_min_index, 2],
                              gt_seq[j, 1], gt_seq[j, 2]))
            TOS_error_time.append(gt_seq[j, 0])

            # error.append(abs(track_list[track_seq][min_index, 1]-gt_seq[j,1]))

    # plt.figure()
    # plt.plot(error_time, PUCK_error)
    # plt.plot(error_time, CL_error)
    # plt.plot(error_time, PF_error)
    # plt.title("Seq #"+str(track_seq))
    # plt.show()
    track_seq = track_seq + 1

np.savetxt("/data/iros_datasets/analysis_percentile/PUCK_moving.txt", PUCK_error_moving)
np.savetxt("/data/iros_datasets/analysis_percentile/PF_moving.txt", PF_error_moving)
np.savetxt("/data/iros_datasets/analysis_percentile/CL_moving.txt", CL_error_moving)
np.savetxt("/data/iros_datasets/analysis_percentile/TOS_moving.txt", TOS_error_moving)

PUCK_mean_error_moving = np.mean(PUCK_error_moving)
PUCK_std_error_moving = np.std(PUCK_error_moving)
PF_mean_error_moving = np.mean(PF_error_moving)
PF_std_error_moving = np.std(PF_error_moving)
CL_mean_error_moving = np.mean(CL_error_moving)
CL_std_error_moving = np.std(CL_error_moving)
TOS_mean_error_moving = np.mean(TOS_error_moving)
TOS_std_error_moving = np.std(TOS_error_moving)
print("PUCK="+str(PUCK_mean_error_moving)+"+-"+str(PUCK_std_error_moving))
print("PF="+str(PF_mean_error_moving)+"+-"+str(PF_std_error_moving))
print("CL="+str(CL_mean_error_moving)+"+-"+str(CL_std_error_moving))
print("TOS="+str(TOS_mean_error_moving)+"+-"+str(TOS_std_error_moving))

alg_names=["PUCK", "PFT", "Cluster", "TOS"]
color_bar = ['blue', 'skyblue', 'darkcyan']
n_items = np.arange(len(alg_names))
mean_comparison_error = [PUCK_mean_error_moving, PF_mean_error_moving, CL_mean_error_moving, TOS_mean_error_moving]
std_comparison_error = [PUCK_std_error_moving, PF_std_error_moving, CL_std_error_moving, TOS_std_error_moving]

fig, ax = plt.subplots()
ax.bar(n_items, mean_comparison_error, yerr=std_comparison_error, align='center', ecolor='black', alpha=0.5, capsize=10)
ax.set_ylabel('Error [pix]')
ax.set_xticks(n_items)
ax.set_xticklabels(alg_names)
# ax.set_title('Coefficent of Thermal Expansion (CTE) of Three Metals')
ax.yaxis.grid(True)

# plt.figure()
# plt.plot(GT_list[0][:,0], GT_list[0][:,2], ls="dashed", linewidth=20)
# plt.plot(PUCK_list[0][:,0], PUCK_list[0][:,2])
# plt.show()

PUCK_succ_rates_moving = []
PF_succ_rates_moving = []
CL_succ_rates_moving = []
TOS_succ_rates_moving = []
num_thresholds = 30
for i in range(num_thresholds):
    PUCK_succ_rates_moving.append(100*(len(np.where(np.array(PUCK_error_moving) < i)[0]) / len(PUCK_error_moving)))
    PF_succ_rates_moving.append(100*(len(np.where(np.array(PF_error_moving) < i)[0]) / len(PF_error_moving)))
    CL_succ_rates_moving.append(100*(len(np.where(np.array(CL_error_moving) < i)[0]) / len(CL_error_moving)))
    TOS_succ_rates_moving.append(100*(len(np.where(np.array(TOS_error_moving) < i)[0]) / len(TOS_error_moving)))

plt.figure()
plt.plot(range(num_thresholds), PUCK_succ_rates_moving, linewidth=3, label="PUCK", color="tab:blue")
plt.plot(range(num_thresholds), PF_succ_rates_moving, linewidth=3, label="PFT", color="tab:orange")
plt.plot(range(num_thresholds), CL_succ_rates_moving,  linewidth=3, label="Cluster", color="tab:green")
plt.plot(range(num_thresholds), TOS_succ_rates_moving,  linewidth=3, label="TOS", color="tab:red")
plt.plot(range(num_thresholds), PUCK_succ_static,  linewidth=3, ls='dashed', color="tab:blue")
plt.plot(range(num_thresholds), PF_succ_static,  linewidth=3, ls='dashed', color="tab:orange")
plt.plot(range(num_thresholds), CL_succ_static,  linewidth=3, ls='dashed', color="tab:green")
plt.plot(range(num_thresholds), TOS_succ_static,  linewidth=3, ls='dashed', color="tab:red")
plt.legend(bbox_to_anchor=(0.65, 0.8), loc='upper left', borderaxespad=0)
plt.xlabel("Threshold [pix]")
plt.ylabel("Percentage of \n valid positions [%]")
plt.show()

# MEAN LATENCY
# PUCK_latency_global = [item for sublist in PUCK_latency for item in sublist]
# PF_latency_global = [item for sublist in PF_latency for item in sublist]
# CL_latency_global = [item for sublist in CL_latency for item in sublist]
#
# PUCK_mean_latency = np.mean(PUCK_latency_global)
# PUCK_std_error_latency = np.std(PUCK_latency_global)
# PF_mean_latency = np.mean(PF_latency_global)
# PF_std_error_latency = np.std(PF_latency_global)
# CL_mean_latency = np.mean(CL_latency_global)
# CL_std_error_latency = np.std(CL_latency_global)
#
# comparison_mean_latency = [PUCK_mean_latency, PF_mean_latency, CL_mean_latency]
# comparison_std_latency = [PUCK_std_error_latency, PF_std_error_latency, CL_std_error_latency]

# fig, ax = plt.subplots()
# ax.bar(n_items, comparison_mean_latency, yerr=comparison_std_latency, align='center', ecolor='red', alpha=0.5, capsize=10)
# ax.set_ylabel('Latency [s]')
# ax.set_xticks(n_items)
# ax.set_xticklabels(alg_names)
# # ax.set_title('Coefficent of Thermal Expansion (CTE) of Three Metals')
# ax.yaxis.grid(True)

# MEAN ERROR: STATIC VS MOVING CAMERA

PUCK_mean_error_both = [PUCK_mean_error_static, PUCK_mean_error_moving]
PF_mean_error_both = [PF_mean_error_static, PF_mean_error_moving]
CL_mean_error_both = [CL_mean_error_static, CL_mean_error_moving]

PUCK_std_error_both = [PUCK_std_error_static, PUCK_std_error_moving]
PF_std_error_both = [PF_std_error_static, PF_std_error_moving]
CL_std_error_both = [CL_std_error_static, CL_std_error_moving]

live_trials = np.arange(2)  # the label locations
width = 0.25  # the width of the bars

fig, ax = plt.subplots()
rects1 = ax.bar(live_trials - width, PUCK_mean_error_both, width, yerr= PUCK_std_error_both, label='PUCK')
rects2 = ax.bar(live_trials, PF_mean_error_both, width, yerr=PF_std_error_both, label='PFT')
rects3 = ax.bar(live_trials + width, CL_mean_error_both, width, yerr= CL_std_error_both,label='CL')

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Mean error')
# ax.set_xlabel('Experiment')
# ax.set_title('Latency')
plt.xticks(live_trials,['Static Camera','Moving Camera'])
ax.legend(loc="upper left")

# ax.bar_label(rects1, padding=3)
# ax.bar_label(rects2, padding=3)
# ax.bar_label(rects3, padding=3)

fig.tight_layout()

plt.show()

