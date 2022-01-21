import numpy as np
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt
import matplotlib
from matplotlib import cm
from scipy import signal
import matplotlib.image as mpimg
import cv2


# matplotlib.use('Agg')


# define normalized 2D gaussian
def gaus2d(x, y, mx, my, sx, sy):
    return 1. / (2. * np.pi * sx * sy) * np.exp(-((x - mx)**2. / (2. * sx**2.) + (y - my)**2. / (2. * sy**2.)))

def rgb2gray(rgb):
    return np.dot(rgb[...,:3], [0.2989, 0.5870, 0.1140])


ellipse = Ellipse(xy=(0,0), width=4, height=4, edgecolor='r', lw=2, facecolor='none')
x = np.linspace(0, 10, 50)
y = np.linspace(0, 10, 50)
x1, y1 = np.meshgrid(x, y)  # get 2D variables instead of 1D
z1 = gaus2d(x1, y1, 5, 5, 2, 2)
z1_copy = z1.copy()
z1 = z1/z1.max()
x2, y2 = np.meshgrid(x, y)  # get 2D variables instead of 1D
z2 = gaus2d(x2, y2, 5, 5, 1.8, 1.8)
z2_copy = z2.copy()
z2 = z2/z2.max()
dog_not_norm = z1 - z2
dog = (z1 - z2)/np.max(z1-z2)
dog[dog<0] = 0

img = cv2.imread('../../../../data/for_edpr_meeting/frame_puck_over_star.png', 0)
res = (304, 240)
img = cv2.resize(img, res)
filter = cv2.imread('../../../../data/for_edpr_meeting/black_hole.png', 0)
fsize = (110, 80)
filter = cv2.resize(filter, fsize)
fig1 = plt.figure()
plt.imshow(img, cmap=plt.get_cmap('gray'))
plt.show()

fig2 = plt.figure()
plt.imshow(dog, cmap=plt.get_cmap('gray'))
plt.show()

img_conv = cv2.filter2D(img, -1, dog)

fig3 = plt.figure()
plt.imshow(img_conv, cmap=plt.get_cmap('gray'))
plt.show()

