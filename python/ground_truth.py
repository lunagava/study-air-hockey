import matplotlib.pyplot as plt
import sys
import os

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
	output = d[0][1] + (x - d[0][0]) * (d[1][1] - d[0][1]) / (d[1][0] - d[0][0])
	return output

def compute_deltaTime(start, end, num_points):
	delta_t = (end-start) / num_points
	return delta_t


filePath = sys.argv[1]

with open(filePath) as csvfile:
	lines = csvfile.readlines()

	count = 0

	for i in range(len(lines)):

		x.append(int(lines[i].split(',')[0]))
		y.append(int(lines[i].split(',')[1]))
		timestamp.append(int(lines[i].split(',')[2]))

		count += 1

		if count == 2:

			count = 0

			data = [x[0], y[0]]
			couple.append(data)
			data = [x[1], y[1]]
			couple.append(data)

			x_new = x[0]

			currentT = timestamp[0]
			deltaT = compute_deltaTime(timestamp[0], timestamp[1], n_points)

			factor = (max(x)-min(x))/n_points
			counter_points = 0

			# new points
			while counter_points <= n_points:

				y_new = interpolation(couple, x_new)

				point = [x_new, y_new]

				left_bb = int(round(x_new - bb_width/2))
				right_bb = int(round(x_new + bb_width/2))
				bottom_bb = int(round(y_new - bb_height/2))
				top_bb = int(round(y_new + bb_height/2))

				out_file.write(str(left_bb)+','+str(bottom_bb)+','+str(right_bb)+','+str(top_bb)+','+str(int(round(currentT)))+'\n')

				if (x[0]>x[1]):
					x_new = x_new - factor
				else:
					x_new = x_new + factor

				currentT = currentT + deltaT

				set_points.append(point)
				counter_points += 1

			xp = [x[0] for x in set_points]
			yp = [x[1] for x in set_points]
			plt.plot(xp, yp, 'ro')
			plt.axis([0, 304, 0, 240])
			plt.show()

			set_points.clear()
			x.clear()
			y.clear()
			timestamp.clear()
			data.clear()
			couple.clear()

out_file.close()
