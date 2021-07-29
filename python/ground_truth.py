import sys
import numpy as np

x = []
y = []
timestamp = []
couple = []
# n_points = 10
bb_width = 10
bb_height = 10
getPeriod = 10  # in ms
line_index = 0
first_time = True

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

def compute_n_points(start, end, period):
    period = (end-start)/period
    return period

filePath = sys.argv[1]

ground_truth = np.genfromtxt(filePath, delimiter=",", names=["x", "y", "timestamp"])

xCoM = ground_truth["x"]
yCoM = ground_truth["y"]
CoM_timestamp = ground_truth["timestamp"]

count = 0

for i in range(len(CoM_timestamp)):

    count += 1

    if not first_time:
        couple.clear()
        data = [xCoM[i-1], yCoM[i-1]]
        couple.append(data)
        data = [xCoM[i], yCoM[i]]
        couple.append(data)

    if first_time:
        if count == 2:
            data = [xCoM[i - 1], yCoM[i - 1]]
            couple.append(data)
            data = [xCoM[i], yCoM[i]]
            couple.append(data)
            first_time = False

    if not first_time:

        x_new = xCoM[i - 1]

        currentT = CoM_timestamp[i - 1]
        # deltaT = compute_deltaTime(CoM_timestamp[i - 1], CoM_timestamp[i], n_points)

        n_points = compute_n_points(CoM_timestamp[i - 1], CoM_timestamp[i], getPeriod)

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

            # out_file.write(str(left_bb) + ',' + str(bottom_bb) + ',' + str(right_bb) + ',' + str(top_bb) + ',' + str(
            #     int(round(currentT))) + ',' + str(1) + '\n')

            out_file.write(str(line_index) + ' ' + str(0.001*int(round(currentT))) + ' ' + str(round(x_new)) + ' ' + str(round(y_new)) + ' ' + str(0.001*int(round(currentT))) + '\n')
            print(str(round(x_new)) + " " + str(round(y_new)) + " " + str(0.001*int(round(currentT))))

            if xCoM[i - 1] > xCoM[i]:
                x_new = x_new - x_factor
            else:
                x_new = x_new + x_factor

            currentT = currentT + getPeriod

            counter_points += 1

            line_index += 1

        x.clear()
        y.clear()
        timestamp.clear()
        data.clear()

out_file.close()
