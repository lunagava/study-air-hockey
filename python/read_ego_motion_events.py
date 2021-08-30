import numpy as np
from PIL import Image
from IPython.display import display

file = np.genfromtxt("/root/.local/share/yarp/contexts/event-driven/egoMotion.tsv", delimiter="\t", names=["x", "y"])

x = file["x"]
y = file["y"]

MyImg = Image.new( 'RGB', (304, 240), "black")
pixels = MyImg.load() # creates the pixel map

for i in range(len(x)):
    pixels[int(x[i]), int(y[i])] = (255, 255, 255)

MyImg.show()
