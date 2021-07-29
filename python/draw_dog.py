import numpy as np
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt
import matplotlib
from matplotlib import cm
from scipy import signal
import matplotlib.image as mpimg


# matplotlib.use('Agg')


# define normalized 2D gaussian
def gaus2d(x, y, mx, my, sx, sy):
    return 1. / (2. * np.pi * sx * sy) * np.exp(-((x - mx)**2. / (2. * sx**2.) + (y - my)**2. / (2. * sy**2.)))

def rgb2gray(rgb):
    return np.dot(rgb[...,:3], [0.2989, 0.5870, 0.1140])


ellipse = Ellipse(xy=(0,0), width=3.6, height=1.8, edgecolor='r', lw=2, facecolor='none')
x = np.linspace(0, 10, 101)
y = np.linspace(0, 10, 101)
x1, y1 = np.meshgrid(x, y)  # get 2D variables instead of 1D
z1 = gaus2d(x1, y1, 5, 5, 2.7, 1.35)
z1_copy = z1.copy()
z1 = z1/z1.max()
x2, y2 = np.meshgrid(x, y)  # get 2D variables instead of 1D
z2 = gaus2d(x2, y2, 5, 5, 0.9, 0.45)
z2_copy = z2.copy()
z2 = z2/z2.max()
dog_not_norm = z1 - z2
dog = (z1 - z2)/np.max(z1-z2)
dog[dog<0] = 0

# path
# path1 = 'image_puck.png'
# img1 = mpimg.imread(path1)
# gray1 = rgb2gray(img1)
# img1 = (np.array(gray1))[0:84, 0:84]
# path2 = 'circle.png'
# img2 = mpimg.imread(path2)
# gray2 = rgb2gray(img2)
# img2 = (np.array(gray1))[0:84, 0:84]
# img_conv = signal.convolve2d(img1, z1)
# # img_product = img1 * img2
#
# # Displaying the image
# fig1 = plt.figure()
#
# plt.imshow(img_conv)
# plt.show()
# fig2 = plt.figure()
# plt.imshow(img)
# plt.show()


fig = plt.figure()
ax1 = fig.add_subplot(3,2,5)
ax1.add_artist(ellipse)
im = ax1.imshow(dog, cmap="viridis", extent=(-5, 5, -5, 5))
ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.title.set_text('dog 2D')
cbar = fig.colorbar(im, ax=ax1)
ax2 = fig.add_subplot(3,2,6,projection='3d')
ax2.contour3D(x, y, dog, 100, cmap=cm.viridis)
ax2.set_xlabel('x')
ax2.set_ylabel('y')
ax2.set_zlabel('z')
ax2.title.set_text('dog 3D')

ax3 = fig.add_subplot(3,2,1)
im1 = ax3.imshow(z1, cmap="viridis", extent=(-5, 5, -5, 5))
ax3.set_xlabel('x')
ax3.set_ylabel('y')
ax3.title.set_text('g1 2D')
ax4 = fig.add_subplot(3,2,2,projection='3d')
ax4.contour3D(x, y, z1, 50, cmap=cm.viridis)
ax4.set_xlabel('x')
ax4.set_ylabel('y')
ax4.set_zlabel('z')
ax4.title.set_text('g1 3D')

ax5 = fig.add_subplot(3,2,3)
im2 = ax5.imshow(z2, cmap="viridis", extent=(-5, 5, -5, 5))
ax5.set_xlabel('x')
ax5.set_ylabel('y')
ax5.title.set_text('g2 2D')
ax6 = fig.add_subplot(3,2,4,projection='3d')
ax6.contour3D(x, y, z2, 50, cmap=cm.viridis)
ax6.set_xlabel('x')
ax6.set_ylabel('y')
ax6.set_zlabel('z')
ax6.title.set_text('g2 3D')
plt.show()
