import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.pyplot import figure
import pandas as pd

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

color1 = '#00BFFF'
color2 = '#00FA9A'
color3 = '#EE82EE'

def findFirstIndexMaxi(arr, low, high):
    i = low
    for i in range(high+1):
        if arr[i] > 200:
            break
    return i

def cutMax(arr, low, high):
    i = low
    for i in range(high+1):
        if arr[i] > 20:
            break
    return i

data_PUCK_0 = np.genfromtxt("../../../../data/workshop/data_final/tau_0.000000_data_PUCK.txt", delimiter=" ", names=["eros_latency", "computation_latency", "tau", "puck_x", "puck_y"])
data_PUCK_5 = np.genfromtxt("../../../../data/workshop/data_final/tau_0.005000_data_PUCK.txt", delimiter=" ", names=["eros_latency", "computation_latency", "tau", "puck_x", "puck_y"])
data_PUCK_10 = np.genfromtxt("../../../../data/workshop/data_final/tau_0.010000_data_PUCK.txt", delimiter=" ", names=["eros_latency", "computation_latency", "tau", "puck_x", "puck_y"])
data_PUCK_15 = np.genfromtxt("../../../../data/workshop/data_final/tau_0.015000_data_PUCK.txt", delimiter=" ", names=["eros_latency", "computation_latency", "tau", "puck_x", "puck_y"])
data_PUCK_20 = np.genfromtxt("../../../../data/workshop/data_final/tau_0.020000_data_PUCK.txt", delimiter=" ", names=["eros_latency", "computation_latency", "tau", "puck_x", "puck_y"])
data_PUCK_25 = np.genfromtxt("../../../../data/workshop/data_final/tau_0.025000_data_PUCK.txt", delimiter=" ", names=["eros_latency", "computation_latency", "tau", "puck_x", "puck_y"])
data_PUCK_30 = np.genfromtxt("../../../../data/workshop/data_final/tau_0.030000_data_PUCK.txt", delimiter=" ", names=["eros_latency", "computation_latency", "tau", "puck_x", "puck_y"])
data_PF = np.genfromtxt("../../../../data/workshop/data_final/data_PF.txt", delimiter=" ", names=["time", "puck_x", "puck_y", "latency"])
data_CL = np.genfromtxt("../../../../data/workshop/data_final/data_CL.txt", delimiter=" ", names=["time", "puck_x", "puck_y", "latency"])

PUCK_latency_0 = data_PUCK_0["eros_latency"][1:]+data_PUCK_0["computation_latency"][1:]
puck_x_0 = data_PUCK_0["puck_x"][1:]
puck_y_0 = data_PUCK_0["puck_y"][1:]
time_0 = [0.001]*len(puck_x_0)
time_0 = np.cumsum(time_0)

PUCK_latency_5 = data_PUCK_5["eros_latency"][1:]+data_PUCK_5["computation_latency"][1:]
puck_x_5 = data_PUCK_5["puck_x"][1:]
puck_y_5 = data_PUCK_5["puck_y"][1:]
time_5 = [0.001]*len(puck_x_5)
time_5 = np.cumsum(time_5)

PUCK_latency_10 = data_PUCK_10["eros_latency"][1:]+data_PUCK_10["computation_latency"][1:]
puck_x_10 = data_PUCK_10["puck_x"][1:]
puck_y_10 = data_PUCK_10["puck_y"][1:]
time_10 = [0.001]*len(puck_x_10)
time_10 = np.cumsum(time_10)

PUCK_latency_15 = data_PUCK_15["eros_latency"][1:]+data_PUCK_15["computation_latency"][1:]
puck_x_15 = data_PUCK_15["puck_x"][1:]
puck_y_15 = data_PUCK_15["puck_y"][1:]
time_15 = [0.001]*len(puck_x_15)
time_15 = np.cumsum(time_15)

PUCK_latency_20 = data_PUCK_20["eros_latency"][1:]+data_PUCK_20["computation_latency"][1:]
puck_x_20 = data_PUCK_20["puck_x"][1:]
puck_y_20 = data_PUCK_20["puck_y"][1:]
time_20 = [0.001]*len(puck_x_20)
time_20 = np.cumsum(time_20)

PUCK_latency_25 = data_PUCK_25["eros_latency"][1:]+data_PUCK_25["computation_latency"][1:]
puck_x_25 = data_PUCK_25["puck_x"][1:]
puck_y_25 = data_PUCK_25["puck_y"][1:]
time_25 = [0.001]*len(puck_x_25)
time_25 = np.cumsum(time_25)

PUCK_latency_30 = data_PUCK_30["eros_latency"][1:]+data_PUCK_30["computation_latency"][1:]
puck_x_30 = data_PUCK_30["puck_x"][1:]
puck_y_30 = data_PUCK_30["puck_y"][1:]
time_30 = [0.001]*len(puck_x_30)
time_30 = np.cumsum(time_30)

PF_latency = data_PF["latency"][1:]
CL_latency = data_CL["latency"][1:]

PUCK_mean_latency_0 = np.mean(PUCK_latency_0)
PUCK_std_error_latency_0 = np.std(PUCK_latency_0)
# print(PUCK_mean_latency_0)
# print(PUCK_std_error_latency_0)
PUCK_mean_latency_5 = np.mean(PUCK_latency_5)
PUCK_std_error_latency_5 = np.std(PUCK_latency_5)
PUCK_mean_latency_10 = np.mean(PUCK_latency_10)
PUCK_std_error_latency_10 = np.std(PUCK_latency_10)
PUCK_mean_latency_15 = np.mean(PUCK_latency_15)
PUCK_std_error_latency_15 = np.std(PUCK_latency_15)
PUCK_mean_latency_20 = np.mean(PUCK_latency_20)
PUCK_std_error_latency_20 = np.std(PUCK_latency_20)
PUCK_mean_latency_25 = np.mean(PUCK_latency_25)
PUCK_std_error_latency_25 = np.std(PUCK_latency_25)
PUCK_mean_latency_30 = np.mean(PUCK_latency_30)
PUCK_std_error_latency_30 = np.std(PUCK_latency_30)

PF_mean_latency = np.mean(PF_latency)
PF_std_error_latency = np.std(PF_latency)
CL_mean_latency = np.mean(CL_latency)
CL_std_error_latency = np.std(CL_latency)

comparison_mean_latency_alg = [PUCK_mean_latency_0, PF_mean_latency, CL_mean_latency]
comparison_std_latency_alg = [PUCK_std_error_latency_0, PF_std_error_latency, CL_std_error_latency]

alg_names=["PUCK", "PFT", "Cluster"]
color_bar = ['tab:blue', 'tab:orange', 'tab:green']
n_items = np.arange(len(alg_names))

# GRAPH 1 - LATENCIES COMPARISON
# fig1, ax1 = plt.subplots()
# ax1.bar(n_items, comparison_mean_latency_alg, yerr=comparison_std_latency_alg, align='center', color='tab:pink', ecolor='red', alpha=0.5, capsize=10)
# ax1.set_ylabel('Latency [s]')
# ax1.set_xticks(n_items)
# ax1.set_xticklabels(alg_names)
# ax1.yaxis.grid(True)
# plt.show()

# GRAPH 2 - CONTROL ERROR OVER TIME - PUCK POS COMPARED TO CENTRE OF THE IMAGE

fixation_x_0 = [320]*len(time_0)
fixation_y_0 = [240]*len(time_0)
fixation_x_5 = [320]*len(time_5)
fixation_y_5 = [240]*len(time_5)
fixation_x_10 = [320]*len(time_10)
fixation_y_10 = [240]*len(time_10)
fixation_x_15 = [320]*len(time_15)
fixation_y_15 = [240]*len(time_15)
fixation_x_20 = [320]*len(time_20)
fixation_y_20 = [240]*len(time_20)
fixation_x_25 = [320]*len(time_25)
fixation_y_25 = [240]*len(time_25)
fixation_x_30 = [320]*len(time_30)
fixation_y_30 = [240]*len(time_30)

error_x_0 = puck_x_0-fixation_x_0
error_y_0 = puck_y_0-fixation_y_0

error_dist_0 = np.sqrt((puck_x_0-fixation_x_0)*(puck_x_0-fixation_x_0)+(puck_y_0-fixation_y_0)*(puck_y_0-fixation_y_0))
error_dist_5 = np.sqrt((puck_x_5-fixation_x_5)*(puck_x_5-fixation_x_5)+(puck_y_5-fixation_y_5)*(puck_y_5-fixation_y_5))
error_dist_10 = np.sqrt((puck_x_10-fixation_x_10)*(puck_x_10-fixation_x_10)+(puck_y_10-fixation_y_10)*(puck_y_10-fixation_y_10))
error_dist_15 = np.sqrt((puck_x_15-fixation_x_15)*(puck_x_15-fixation_x_15)+(puck_y_15-fixation_y_15)*(puck_y_15-fixation_y_15))
error_dist_20 = np.sqrt((puck_x_20-fixation_x_20)*(puck_x_20-fixation_x_20)+(puck_y_20-fixation_y_20)*(puck_y_20-fixation_y_20))
error_dist_25 = np.sqrt((puck_x_25-fixation_x_25)*(puck_x_25-fixation_x_25)+(puck_y_25-fixation_y_25)*(puck_y_25-fixation_y_25))
error_dist_30 = np.sqrt((puck_x_30-fixation_x_30)*(puck_x_30-fixation_x_30)+(puck_y_30-fixation_y_30)*(puck_y_30-fixation_y_30))

mean_error_dist_0 = np.mean(error_dist_0)
mean_error_dist_5 = np.mean(error_dist_5)
mean_error_dist_10 = np.mean(error_dist_10)
mean_error_dist_15 = np.mean(error_dist_15)
mean_error_dist_20 = np.mean(error_dist_20)
mean_error_dist_25 = np.mean(error_dist_25)
mean_error_dist_30 = np.mean(error_dist_30)

std_error_dist_0 = np.std(error_dist_0)
std_error_dist_5 = np.std(error_dist_5)
std_error_dist_10 = np.std(error_dist_10)
std_error_dist_15 = np.std(error_dist_15)
std_error_dist_20 = np.std(error_dist_20)
std_error_dist_25 = np.std(error_dist_25)
std_error_dist_30 = np.std(error_dist_30)

plt.figure(figsize=(10, 6))
plt.plot(time_0, puck_x_0, c=color1, linewidth=2, label="$u_{puck}$")
plt.plot(time_0, fixation_x_0, c=color2, linewidth=2,  label="$u_{fixation}$")
plt.plot(time_0, puck_y_0, c=color3, linewidth=2,  label="$v_{puck}$")
plt.plot(time_0, fixation_y_0, c="tab:purple", linewidth=2,  label="$v_{fixation}$")
plt.xlabel("Time [s]")
plt.ylabel("Position [pxl]")
plt.legend( fontsize=13, bbox_to_anchor=(0,1.02,1,1), loc="lower left", mode="expand", borderaxespad=0, ncol=4)
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(time_0, error_x_0, c=color1, linewidth=2, label="along x")
plt.plot(time_0, error_y_0, c=color3, linewidth=2, label="along y")
plt.xlabel("Time [s]")
plt.ylabel("Error [pxl]")
plt.legend(bbox_to_anchor=(0.8,0.8), borderaxespad=0)
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(time_0, error_dist_0, c=color1, linewidth=1)
plt.xlabel("Time [s]")
plt.ylabel("Distance Error [pxl]")
plt.show()


# GRAPH 3 - ADDING DELAY IN THE CONTROL LOOP

comparison_mean_latency_tau = [PUCK_mean_latency_0, PUCK_mean_latency_5, PUCK_mean_latency_10, PUCK_mean_latency_15, PUCK_mean_latency_20, PUCK_mean_latency_25, PUCK_mean_latency_30]
comparison_std_latency_tau = [PUCK_std_error_latency_0, PUCK_std_error_latency_5, PUCK_std_error_latency_10, PUCK_std_error_latency_15, PUCK_std_error_latency_20, PUCK_std_error_latency_25, PUCK_std_error_latency_30]

tau=["0", "5", "10", "15", "20", "25", "30"]
n_items_tau = np.arange(len(tau))
medianprops = dict(color='red')

fig = plt.figure(figsize=(10, 6))
bplot = plt.boxplot(positions=[1, 2, 3, 4, 5, 6, 7], x=[PUCK_latency_0, PUCK_latency_5, PUCK_latency_10, PUCK_latency_15, PUCK_latency_20, PUCK_latency_25, PUCK_latency_30], showfliers=False,
                    widths=0.7,patch_artist=True, medianprops=medianprops)
plt.xticks([1, 2, 3, 4, 5, 6, 7], tau)
# fill with colors
colors = ['#ABCDEF', '#ABCDEF', '#ABCDEF', '#ABCDEF', '#ABCDEF', '#ABCDEF', '#ABCDEF']
for patch, color in zip(bplot['boxes'], colors):
    patch.set_facecolor(color)
plt.yscale("log")
plt.ylabel("Latency Quartilies [s]")
plt.xlabel(r'Delay $\tau_0$ [s]')
plt.show()

comparison_mean_error_tau = [mean_error_dist_0, mean_error_dist_5, mean_error_dist_10, mean_error_dist_15, mean_error_dist_20, mean_error_dist_25, mean_error_dist_30]
comparison_std_error_tau = [std_error_dist_0, std_error_dist_5, std_error_dist_10, std_error_dist_15, std_error_dist_20, std_error_dist_25, std_error_dist_30]

fig5, ax5 = plt.subplots(figsize=(10,6))
ax5.bar(n_items_tau, comparison_mean_error_tau, yerr=comparison_std_error_tau, align='center', color='tab:pink',ecolor='red', alpha=0.5, capsize=10)
ax5.set_ylabel('Mean error [pxl]')
ax5.set_xlabel(r'Delay $\tau_0$ [s]')
ax5.set_xticks(n_items_tau)
ax5.set_xticklabels(tau)
ax5.yaxis.grid(True)
plt.show()


firstMax_20 = findFirstIndexMaxi(error_dist_20, 0, len(error_dist_20))
firstMax_25 = findFirstIndexMaxi(error_dist_25, 0, len(error_dist_25))
firstMax_30 = findFirstIndexMaxi(error_dist_30, 0, len(error_dist_30))

index_20 = cutMax(time_20, 0, len(time_20))
index_25 = cutMax(time_25, 0, len(time_25))
index_30 = cutMax(time_30, 0, len(time_30))

fig6, axs6 = plt.subplots(3,1, sharex=True, sharey=True, figsize=(10,6))
axs6[0].plot(time_20[0:index_20], error_dist_20[0:index_20], c=color1, linewidth=2, label=r"$\tau_0$ = 20")
axs6[0].legend(loc="upper left")
axs6[0].axvline(x=time_20[firstMax_20])
axs6[1].plot(time_25[0:index_25], error_dist_25[0:index_25], c=color1, linewidth=2, label=r"$\tau_0$ = 25")
axs6[1].legend(loc="upper left")
axs6[1].axvline(x=time_25[firstMax_25])
axs6[2].plot(time_30[0:index_30], error_dist_30[0:index_30], c=color1, linewidth=2, label=r"$\tau_0$ = 30")
axs6[2].legend(loc="upper left")
axs6[2].axvline(x=time_30[firstMax_30])
plt.setp(axs6[1], ylabel='Distance Error [pxl]')
plt.setp(axs6[-1], xlabel='Time [s]')
plt.legend()
plt.show()