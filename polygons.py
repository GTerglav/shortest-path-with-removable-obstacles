import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

# start = np.array([1, 3])
# goal = np.array([10, 1])
# obstacles = [[[2, 2], [3, 3], [3, 4], [2, 5]],  # Obstacle 1
#              [[6, 3], [7, 3], [7, 4], [6, 4]],  # Obstacle 2
#              [[4, 1], [5, 2], [4, 3], [3, 2]]]  # Obstacle 3
# costs = [8, 5, 10]  # Costs corresponding to each obstacle
# budget = 7
# epsilon = 1

start = [-1, 1]
goal = [5, 0]
obstacles = [
    [[2, 1], [2, 2], [1, 2], [0.5, 1.5]],
   [[4, 1], [4, 1], [4, 0], [3, 0], [2.5, 1]],
   [[0, 0], [1, 0], [1, 1], [0, 1], [-0.5, 0.5]],
]
costs = [3,3,3]
budget = 1
epsilon = 1