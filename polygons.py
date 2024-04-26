import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

# Define the vertices for multiple polygons
points = np.array([[-1, 1], [5, 0]])

polygons = [
    np.array([[2, 1], [2, 3], [2, 2], [1, 2], [0.5, 1.5]]),
    np.array([[4, 1], [4, 1], [4, 0], [3, 0], [2.5, 1]]),
    np.array([[0, 0], [1, 0], [1, 1], [0, 1], [-0.5, 0.5]]),
]

fig, ax = plt.subplots()

# Add each polygon to the plot
for vertices in polygons:
    p = Polygon(vertices, facecolor='k')
    ax.add_patch(p)

ax.scatter(points[:, 0], points[:, 1], color='r', label='Points')

ax.set_xlim([-2, 6])
ax.set_ylim([-1, 5])
plt.show()