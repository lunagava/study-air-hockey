import numpy as np
import matplotlib
import matplotlib.pyplot as plt

table = np.genfromtxt("../../../../data/table/table_Sim_2cm.tsv", delimiter="\t", names=["y", "torso_pitch", "torso_roll", "torso_yaw", "arm_shoulder_pitch", "arm_shoulder_roll", "arm_shoulder_yaw", "arm_elbow", "arm_wrist_prosup", "arm_wrist_pitch", "arm_wrist_yaw", "head_pitch", "head_roll", "head_yaw", "eyes_tilt", "eyes_vers", "eyes_verg"])

fig1 = plt.figure()
plt.subplot(2,1,1)
plt.plot(table['y'], table['arm_shoulder_pitch'], color='b', alpha=0.7, label='shoulder pitch', linewidth=2)
plt.plot(table['y'], table['arm_shoulder_roll'], color='r', alpha=0.7, label='shoulder roll', linewidth=2)
plt.plot(table['y'], table['arm_shoulder_yaw'], color='m', alpha=0.7, label='shoulder yaw', linewidth=2)
plt.plot(table['y'], table['arm_elbow'], color='k', alpha=0.7, label='elbow', linewidth=2)
plt.plot(table['y'], table['arm_wrist_prosup'], color='y', alpha=0.7, label='wrist prosup', linewidth=2)
plt.plot(table['y'], table['arm_wrist_pitch'], color='g', alpha=0.7, label='wrist pitch', linewidth=2)
plt.plot(table['y'], table['arm_wrist_yaw'], color='r', alpha=0.4, label='wrist yaw', linewidth=2)
# plt.plot(table['y'], table['head_pitch'], color='b', alpha=0.7, label='head pitch', linewidth=2)
# plt.plot(table['y'], table['head_roll'], color='r', alpha=0.7, label='head roll', linewidth=2)
# plt.plot(table['y'], table['head_yaw'], color='m', alpha=0.7, label='head yaw', linewidth=2)
# plt.plot(table['y'], table['eyes_tilt'], color='k', alpha=0.7, label='eyes tilt', linewidth=2)
# plt.plot(table['y'], table['eyes_vers'], color='g', alpha=0.9, label='eyes vers', linewidth=2)
# plt.plot(table['y'], table['eyes_verg'], color='r', alpha=0.4, label='eyes verg', linewidth=2)
# plt.plot(table['y'], table['torso_pitch'], color='b', alpha=0.7, label='torso pitch', linewidth=2)
# plt.plot(table['y'], table['torso_roll'], color='r', alpha=0.7, label='torso roll', linewidth=2)
# plt.plot(table['y'], table['torso_yaw'], color='m', alpha=0.7, label='torso yaw', linewidth=2)

plt.xticks(fontsize=14)
plt.yticks(fontsize=14)

leg = plt.legend(fontsize=14, loc='lower center', bbox_to_anchor=(0.5, -1.3), ncol=2)

plt.title('Arm Joints', fontsize=20)
plt.xlabel('End Effector y position [m]', fontsize=18)
plt.ylabel('[deg]', fontsize=16)
plt.show()

