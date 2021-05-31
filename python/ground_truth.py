import matplotlib.pyplot as plt

x = []
y = []
timestamp = []
n_points = 20
set_points = []
couple = []

def interpolation(d, x):
	output = int(d[0][1] + (x - d[0][0]) * (d[1][1] - d[0][1]) / (d[1][0] - d[0][0]))
	return output


with open('ground_truth.csv') as csvfile:
	lines = csvfile.readlines()

	count = 0

	for i in range(len(lines)):

		x.append(int(lines[i].split(',')[0]))
		y.append(int(lines[i].split(',')[1]))
		timestamp.append(int(lines[i].split(',')[2]))

		count += 1

		if (count == 2):

			count = 0

			data = [x[0], y[0]]
			couple.append(data)
			data = [x[1], y[1]]
			couple.append(data)

			x_new = min(x)

			factor = (max(x)-min(x))/n_points

			# new points
			while (x_new <= max(x)):
				x_new = x_new + factor

				y_new = interpolation(couple, x_new)

				point = [x_new, y_new]

				set_points.append(point)

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

