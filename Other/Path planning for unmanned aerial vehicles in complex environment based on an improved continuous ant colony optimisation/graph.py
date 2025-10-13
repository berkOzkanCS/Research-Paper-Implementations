import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Read threats
THREATS = []
with open("threats.txt") as f:
    for line in f:
        x, y, z, R, h = map(float, line.split())
        THREATS.append((x, y, z, R, h))

# Read path
path = []
with open("path_output.txt") as f:
    for line in f:
        x, y, z = map(float, line.split())
        path.append((x, y, z))

# Plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot path
px, py, pz = zip(*path)
ax.plot(px, py, pz, 'r-o', label='Best Path')

# Plot cylinders
import numpy as np
for x0, y0, z0, R, h in THREATS:
    # create cylinder
    theta = np.linspace(0, 2*np.pi, 30)
    z = np.linspace(z0, z0 + h, 10)
    theta, z = np.meshgrid(theta, z)
    X = R * np.cos(theta) + x0
    Y = R * np.sin(theta) + y0
    Z = z
    ax.plot_surface(X, Y, Z, color='b', alpha=0.3)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('UAV Path and Threats')
ax.legend()
plt.show()
