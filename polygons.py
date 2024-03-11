import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

y = np.array([[1,1], [2,1], [2,2], [1,2], [0.5,1.5]])

p = Polygon(y, facecolor = 'k')

fig,ax = plt.subplots()

ax.add_patch(p)
ax.set_xlim([0,3])
ax.set_ylim([0,3])
plt.show()