import sys
import numpy as np
import csv

bb_width = 40
bb_height = 40

out_file = open("COM_new.csv", "w")

filePath_computed_COM = sys.argv[1]

COM_pix = np.genfromtxt(filePath_computed_COM, delimiter=",", names=["x", "y", "timestamp"])

xCoM = COM_pix["x"]
yCoM = COM_pix["y"]
CoM_timestamp = COM_pix["timestamp"]

for i in range(len(xCoM)):

    left_bb = int(round(xCoM[i] - bb_width / 2))
    right_bb = int(round(xCoM[i] + bb_width / 2))
    bottom_bb = int(round(yCoM[i] - bb_height / 2))
    top_bb = int(round(yCoM[i] + bb_height / 2))
    label = 2

    if left_bb < 0:
        left_bb = 0

    if right_bb < 0:
        right_bb = 0

    if bottom_bb < 0:
        bottom_bb = 0

    if top_bb < 0:
        top_bb = 0

    out_file_writer = csv.writer(out_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    out_file_writer.writerow([left_bb, bottom_bb, right_bb, top_bb, round(CoM_timestamp[i]), label])
