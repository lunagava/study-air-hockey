
import matplotlib, numpy as np
import matplotlib.pyplot as plt
import os, numpy as np
from tqdm import tqdm
params = {'xtick.labelsize': 24,
          'ytick.labelsize': 24,
          'font.size': 13,
          'figure.autolayout': True,  # similar to tight_layout
          'figure.figsize': [10, 7],  # ratio 1.6 is pleasant
          'axes.titlesize': 13,
          'axes.labelsize': 34,
          'lines.linewidth': 2,
          'lines.markersize': 4,
          'legend.fontsize': 16}

matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
plt.style.use(params)

labels = ['PUCK\nstatic', 'PF\nstatic', 'CL\nstatic', 'PUCK\nmoving', 'PF\nmoving', 'CL\nmoving']
medianprops = dict(color='white')

PUCK_static = np.loadtxt("/data/iros_datasets/analysis_percentile/PUCK_static.txt")
PF_static   = np.loadtxt("/data/iros_datasets/analysis_percentile/PF_static.txt")
CL_static   = np.loadtxt("/data/iros_datasets/analysis_percentile/CL_static.txt")
PUCK_moving = np.loadtxt("/data/iros_datasets/analysis_percentile/PUCK_moving.txt")
PF_moving   = np.loadtxt("/data/iros_datasets/analysis_percentile/PF_moving.txt")
CL_moving   = np.loadtxt("/data/iros_datasets/analysis_percentile/CL_moving.txt")

data_PUCK_0 = np.genfromtxt("../../../../data/workshop/data_final/tau_0.000000_data_PUCK.txt", delimiter=" ", names=["eros_latency", "computation_latency", "tau", "puck_x", "puck_y"])
data_PF = np.genfromtxt("../../../../data/workshop/data_final/data_PF.txt", delimiter=" ", names=["time", "puck_x", "puck_y", "latency"])
data_CL = np.genfromtxt("../../../../data/workshop/data_final/data_CL.txt", delimiter=" ", names=["time", "puck_x", "puck_y", "latency"])

PUCK_latency_0 = data_PUCK_0["eros_latency"][1:]+data_PUCK_0["computation_latency"][1:]
PF_lat_new = data_PF["latency"]
CL_lat_new = data_CL["latency"]

# PUCK_static_25 = np.percentile(PUCK_static, 25)
# PUCK_static_50 = np.percentile(PUCK_static, 50)
# PUCK_static_75 = np.percentile(PUCK_static, 75)
#
# PF_static_25 = np.percentile(PF_static, 25)
# PF_static_50 = np.percentile(PF_static, 50)
# PF_static_75 = np.percentile(PF_static, 75)
#
# CL_static_25 = np.percentile(CL_static, 25)
# CL_static_50 = np.percentile(CL_static, 50)
# CL_static_75 = np.percentile(CL_static, 75)

fig = plt.figure()
bplot = plt.boxplot(positions=[1, 2, 3, 5, 6, 7], x=[PUCK_static, PF_static, CL_static, PUCK_moving, PF_moving, CL_moving], showfliers=False,
                    patch_artist=True, medianprops=medianprops)
plt.xticks([1, 2, 3, 5, 6, 7], labels, rotation=45)
# fill with colors
colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:blue', 'tab:orange', 'tab:green']
for patch, color in zip(bplot['boxes'], colors):
    patch.set_facecolor(color)
plt.yscale("log")
plt.ylabel("Error quartiles [pxl]", color='k')
plt.show()

quart_vec=[PUCK_static, PF_static, CL_static, PUCK_moving, PF_moving, CL_moving]

fig, ax1 = plt.subplots()
algs = ['PUCK', 'PF', 'CL']
ax1.set_ylabel('Error quartiles', color='k')
res1 = ax1.boxplot(positions=np.arange(6), x=[PUCK_static, PF_static, CL_static, PUCK_moving, PF_moving, CL_moving], showfliers=False,
                    patch_artist=True, medianprops=medianprops)
plt.yscale("log")
for element in ['boxes', 'whiskers', 'fliers', 'means', 'medians', 'caps']:
    plt.setp(res1[element], color='k')

colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:blue', 'tab:orange', 'tab:green']
for patch, color in zip(res1['boxes'], colors):
    patch.set_facecolor(color)
ax1.set_xlim([-0.55, 9.55])
ax1.set_xticks(np.arange(10))
labels = ['PUCK static', 'PF static', 'CL static', 'PUCK moving', 'PF moving', 'CL moving', 'PF static', 'CL static', 'PF moving', 'CL moving']
ax1.set_xticklabels(labels, rotation=90)
plt.show()

fig, ax2 = plt.subplots()
PF_time_to_fail_static=1
CL_time_to_fail_static=6.6
PF_time_to_fail_moving=4
CL_time_to_fail_moving=5
width = 0.4
ax2.set_ylabel('Time to failure', color='k')
plt.plot(10,600,marker='x', color='tab:blue')
plt.plot(30,600,marker='x', color='tab:blue')
rects1 = ax2.bar(0, PF_time_to_fail_static, width,  color='tab:orange')
rects2 = ax2.bar(1, CL_time_to_fail_static, width, color='tab:green')
rects3 = ax2.bar(2, PF_time_to_fail_moving, width,  color='tab:orange')
rects4 = ax2.bar(3, CL_time_to_fail_moving, width, color='tab:green')
rects5 = ax2.bar(4, PIM_time_to_fail_static, width,  color='tab:purple')
rects6 = ax2.bar(5, TOS_time_to_fail_static, width, color='tab:red')
rects7 = ax2.bar(6, PIM_time_to_fail_moving, width,  color='tab:purple')
rects8 = ax2.bar(7, TOS_time_to_fail_moving, width, color='tab:red')
# plt.xticks([6, 7, 8], labels, rotation=45)
# ax2.bar(6, time_to_fail_vec, 0.4, align='center', color='tab:blue', ecolor='k',edgecolor='black', alpha=0.5, capsize=10)

labels = ['PUCK static', 'PF static', 'CL static', 'PUCK moving', 'PF moving', 'CL moving', 'PF static', 'CL static', 'PF moving', 'CL moving']
ax2.set_xticklabels(labels, rotation=90)
fig.tight_layout()  # otherwise the right y-label is slightly clipped
plt.show()


live_test_path = "../../../../data/iros_datasets/live_test"
live_test_files = sorted([traj for traj in os.listdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), live_test_path)) if traj.endswith('.txt')])
PUCK_latency_list=[]
PF_latency_list=[]
CL_latency_list=[]
for t in tqdm(live_test_files, "Loading live test data"):

    if "puck" in t:
        PUCK_live_test_traj = np.loadtxt(os.path.join(live_test_path, t), delimiter=" ")[:, :4]
        PUCK_latency_list.append(PUCK_live_test_traj[:,3])
    elif "vPFT" in t:
        PF_live_test_traj = np.loadtxt(os.path.join(live_test_path, t), delimiter=" ")[:, :4]
        PF_latency_list.append(PF_live_test_traj[:,3])
    elif "clust" in t:
        CL_live_test_traj = np.loadtxt(os.path.join(live_test_path, t), delimiter=" ")[:, :4]
        CL_latency_list.append(CL_live_test_traj[:,3])

PUCK_flat_list = [item for sublist in PUCK_latency_list for item in sublist]
PF_flat_list = [item for sublist in PF_latency_list for item in sublist]
CL_flat_list = [item for sublist in CL_latency_list for item in sublist]

PUCK_mean_lat = np.mean(PUCK_flat_list)
PF_mean_lat = np.mean(PF_flat_list)
CL_mean_lat = np.mean(CL_flat_list)

n_items=np.arange(3)
alg_latency = [PUCK_mean_lat, PF_mean_lat, CL_mean_lat]
alg_latency_new = [np.mean(PUCK_latency_0), np.mean(PF_lat_new), np.mean(CL_lat_new)]
alg_error = [np.mean(PUCK_moving), np.mean(PF_moving), np.mean(CL_moving)]
width = 0.3

labels = ['PUCK', "PF", "Cluster"]

fig, ax = plt.subplots()
rects1 = ax.bar(0, PUCK_mean_lat, width,  color='tab:blue')
rects2 = ax.bar(1, PF_mean_lat, width, color='tab:orange')
rects3 = ax.bar(2, CL_mean_lat, width, color='tab:green')

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Mean Latency [s]')
# ax.set_xlabel('\n Algorithm')
plt.xticks([0, 1, 2], labels, rotation=45)
plt.yscale("log")
# plt.ylim([0,np.power(10,-2)])

# ax.set_title('Latency')
# ax.set_xticks(live_trials, labels

# ax.legend()

ax.bar_label(rects1, padding=3)
ax.bar_label(rects2, padding=3)
ax.bar_label(rects3, padding=3)

fig.tight_layout()

plt.show()

fig3, ax1 = plt.subplots(figsize=(10,6))
# ax1.set_xlabel('X-axis')
ax1.set_ylabel('Mean Latency [s]', color='tab:blue')
ax1.bar(n_items, alg_latency_new,width, align='center', color='tab:blue', ecolor='k',edgecolor='black', alpha=0.5, capsize=10)
ax1.set_yscale("log")
ax1.tick_params(axis='y', labelcolor='tab:blue')

# Adding Twin Axes

ax2 = ax1.twinx()
ax2.set_ylabel('Mean Accuracy [pxl]', color='tab:pink')
ax2.bar(n_items+width, alg_error, width, align='center', color='tab:pink', ecolor='k', edgecolor='black', alpha=0.5, capsize=10)
ax2.tick_params(axis='y', labelcolor='tab:pink')

plt.xticks(n_items + width / 2, ('PUCK', 'PF', 'Cluster'))
plt.show()


# medianprops = dict(color='white')
#
# labels = ['PUCK static', 'PF static', 'CL static']
#
# fig1 = plt.figure()
# bplot = plt.boxplot([PUCK_static, PF_static, CL_static], showfliers=False, patch_artist=True, medianprops=medianprops)
# plt.xticks([1, 2, 3], labels, rotation=45)
# # fill with colors
# colors = ['tab:blue', 'tab:orange', 'tab:green']
# for patch, color in zip(bplot['boxes'], colors):
#     patch.set_facecolor(color)
# plt.yscale("log")
# plt.show()
#
#
# labels = ['PUCK moving', 'PF moving', 'CL moving']
#
# fig2 = plt.figure()
# bplot = plt.boxplot([PUCK_moving, PF_moving, CL_moving], showfliers=False, patch_artist=True, medianprops=medianprops)
# plt.xticks([1, 2, 3], labels, rotation=45)
# # fill with colors
# colors = ['tab:blue', 'tab:orange', 'tab:green']
# for patch, color in zip(bplot['boxes'], colors):
#     patch.set_facecolor(color)
# plt.yscale("log")
# plt.show()

cameras = ['Frame-based camera', 'Event-camera']

mean_frame_lat = 93
mean_event_lat = 10
std_frame_lat = 3.5
std_event_lat = 0.15

fig5, ax5 = plt.subplots()
rects1 = ax5.bar(0, mean_frame_lat, yerr=std_frame_lat, width=0.5, color='tab:olive')
rects2 = ax5.bar(1, mean_event_lat, yerr=std_event_lat, width=0.5, color='tab:cyan')

# Add some text for labels, title and custom x-axis tick labels, etc.
ax5.set_ylabel('Sensor Latency [ms]')
# ax.set_xlabel('\n Algorithm')
plt.xticks([0, 1], cameras)

plt.show()

PUCK_mean_lat = 0.01
PF_mean_lat = 0.03
CL_mean_lat = 0.0001
PIM_mean_lat = 0.001
TOS_mean_lat = 0.01

lab_alg = ["PUCK", "PF", "CL", "PIM", "TOS"]

fig, ax = plt.subplots()
rects1 = ax.bar(0, PUCK_mean_lat, width,  color='tab:blue')
rects2 = ax.bar(1, PF_mean_lat, width, color='tab:orange')
rects3 = ax.bar(2, CL_mean_lat, width, color='tab:green')
rects4 = ax.bar(3, PIM_mean_lat, width,  color='tab:purple')
rects5 = ax.bar(4, TOS_mean_lat, width, color='tab:red')

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Mean Latency [s]')
plt.xticks([0, 1, 2, 3, 4], lab_alg, rotation=45)
plt.yscale("log")
# plt.ylim([0,np.power(10,-2)])

# ax.set_title('Latency')
# ax.set_xticks(live_trials, labels

# ax.legend()

ax.bar_label(rects1, padding=3)
ax.bar_label(rects2, padding=3)
ax.bar_label(rects3, padding=3)
ax.bar_label(rects4, padding=3)
ax.bar_label(rects5, padding=3)

fig.tight_layout()

plt.show()