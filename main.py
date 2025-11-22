import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Plot a simple 3D coordinate frame
origin = np.array([0, 0, 0])
X = np.array([1, 0, 0])
Y = np.array([0, 1, 0])
Z = np.array([0, 0, 1])

ax.quiver(*origin, *X, color='r', length=1)
ax.quiver(*origin, *Y, color='g', length=1)
ax.quiver(*origin, *Z, color='b', length=1)

ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([-2, 2])

plt.show()
