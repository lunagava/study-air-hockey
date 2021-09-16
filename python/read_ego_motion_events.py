import numpy as np
from PIL import Image
from IPython.display import display
import matplotlib.pyplot as plt

A = []
with open('/code/luna/study-air-hockey/heat_map.txt', 'r') as f:
    for line in f:
        A.append(list(map(int, line.split())))

max_value, max_index = max((x, (i, j))
                           for i, row in enumerate(A)
                           for j, x in enumerate(row))

print(max_value)

fig = plt.figure()
im = plt.imshow(A, cmap="gist_stern")

# plt.gca().invert_xaxis()
plt.gca().invert_yaxis()
plt.xticks(rotation=90)
plt.yticks(rotation=90)
plt.ylabel("x")
plt.xlabel("y")
cbar = fig.colorbar(im, orientation="vertical")
plt.show()

file = np.genfromtxt("/root/.local/share/yarp/contexts/event-driven/egoMotion_50.tsv", delimiter="\t", names=["x", "y"])

x = file["x"]
y = file["y"]

MyImg = Image.new('RGB', (304, 240), "black")
pixels = MyImg.load()  # creates the pixel map

for i in range(len(x)):
    pixels[int(x[i]), int(y[i])] = (255, 255, 255)

MyImg.show()

MyImg.save("events_suppressed.png")

