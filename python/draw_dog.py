import numpy as np
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt
from matplotlib import cm
from scipy.stats import multivariate_normal
# define normalized 2D gaussian
def gaus2d(x, y, mx, my, sx, sy):
    return 1. / (2. * np.pi * sx * sy) * np.exp(-((x - mx)**2. / (2. * sx**2.) + (y - my)**2. / (2. * sy**2.)))


ellipse = Ellipse(xy=(0,0), width=3.6, height=1.8, edgecolor='r', lw=2, facecolor='none')
x = np.linspace(-5, 5, 100)
y = np.linspace(-5, 5, 100)
x1, y1 = np.meshgrid(x, y)  # get 2D variables instead of 1D
z1 = gaus2d(x1, y1, 0, 0, 2.7, 1.35)
z1 = z1/z1.max()
x2, y2 = np.meshgrid(x, y)  # get 2D variables instead of 1D
z2 = gaus2d(x2, y2, 0, 0, 0.9, 0.45)
z2 = z2/z2.max()
dog = 2*(z1 - z2)/np.max(z1-z2)
dog[dog<0] = 0
fig = plt.figure()
ax1 = fig.add_subplot(1,2,1)
ax1.add_artist(ellipse)
im = ax1.imshow(dog, cmap="viridis", extent=(-5, 5, -5, 5))
ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.title.set_text('dog 2D')
cbar = fig.colorbar(im, ax=ax1)
ax2 = fig.add_subplot(1,2,2,projection='3d')
ax2.contour3D(x, y, dog, 100, cmap=cm.viridis)
ax2.set_xlabel('x')
ax2.set_ylabel('y')
ax2.set_zlabel('z')
ax2.title.set_text('dog 3D')
# ax3 = fig.add_subplot(3,2,1)
# ax3.contourf(x, y, z1, cmap=cm.viridis)
# ax3.set_xlabel('x')
# ax3.set_ylabel('y')
# ax3.title.set_text('g1 2D')
# ax4 = fig.add_subplot(3,2,2,projection='3d')
# ax4.contour3D(x, y, z1, 50, cmap=cm.viridis)
# ax4.set_xlabel('x')
# ax4.set_ylabel('y')
# ax4.set_zlabel('z')
# ax4.title.set_text('g1 3D')
#
# ax5 = fig.add_subplot(3,2,3)
# ax5.contourf(x, y, z2, cmap=cm.viridis)
# ax5.set_xlabel('x')
# ax5.set_ylabel('y')
# ax5.title.set_text('g2 2D')
# ax6 = fig.add_subplot(3,2,4,projection='3d')
# ax6.contour3D(x, y, z2, 50, cmap=cm.viridis)
# ax6.set_xlabel('x')
# ax6.set_ylabel('y')
# ax6.set_zlabel('z')
# ax6.title.set_text('g2 3D')
plt.show()

