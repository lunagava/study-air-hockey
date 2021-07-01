import matplotlib.pyplot as plt
import sys
import numpy as np

x = []
y = []
timestamp = []
set_points = []
couple = []
n_points = 100
bb_width = 10
bb_height = 10

out_file = open("points.csv", "w")


def interpolation(d, x):
    if (d[1][0] - d[0][0]) != 0:
        output = d[0][1] + (x - d[0][0]) * (d[1][1] - d[0][1]) / (d[1][0] - d[0][0])
    else:
        output = 0
    return output


def compute_deltaTime(start, end, num_points):
    delta_t = (end - start) / num_points
    return delta_t


filePath = sys.argv[1]

ground_truth = np.genfromtxt(filePath, delimiter=",", names=["x", "y", "timestamp"])

xCoM = ground_truth["x"]
yCoM = ground_truth["y"]
CoM_timestamp = ground_truth["timestamp"]

count = 0

for i in range(len(CoM_timestamp)):

    count += 1

    if count == 2:

        count = 0

        data = [xCoM[i - 1], yCoM[i - 1]]
        couple.append(data)
        data = [xCoM[i], yCoM[i]]
        couple.append(data)

        print(data)

        x_new = xCoM[i - 1]

        currentT = CoM_timestamp[i - 1]
        deltaT = compute_deltaTime(CoM_timestamp[i - 1], CoM_timestamp[i], n_points)

        x_factor = abs(xCoM[i - 1] - xCoM[i]) / n_points
        if (couple[1][0] - couple[0][0]) == 0:
            y_factor = abs(yCoM[i - 1] - yCoM[i]) / n_points

        counter_points = 0

        # new points
        while counter_points <= n_points:

            if (couple[1][0] - couple[0][0]) != 0:
                y_new = interpolation(couple, x_new)
                point = [x_new, y_new]
            else:
                if yCoM[i - 1] > yCoM[i]:
                    point = [couple[0][0], y_new - y_factor]
                else:
                    point = [couple[0][0], y_new + y_factor]

            left_bb = int(round(x_new - bb_width / 2))
            right_bb = int(round(x_new + bb_width / 2))
            bottom_bb = int(round(y_new - bb_height / 2))
            top_bb = int(round(y_new + bb_height / 2))

            out_file.write(str(left_bb) + ',' + str(bottom_bb) + ',' + str(right_bb) + ',' + str(top_bb) + ',' + str(
                int(round(currentT))) + ',' + str(1) + '\n')

            if xCoM[i - 1] > xCoM[i]:
                x_new = x_new - x_factor
            else:
                x_new = x_new + x_factor

            currentT = currentT + deltaT

            set_points.append(point)
            counter_points += 1

        # xp = [x[0] for x in set_points]
        # yp = [x[1] for x in set_points]
        # plt.plot(xp, yp, 'ro')
        # plt.axis([0, 304, 0, 240])
        # plt.show()

        set_points.clear()
        x.clear()
        y.clear()
        timestamp.clear()
        data.clear()
        couple.clear()

out_file.close()
