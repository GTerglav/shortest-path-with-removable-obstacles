import math
from matplotlib import pyplot as plt
import numpy as np
import random

from helper import distance


class problemParameters:
    def __init__(self, start, goal, obstacles, costs, budget, epsilon):
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.costs = costs
        self.budget = budget
        self.epsilon = epsilon


problemSimple = problemParameters(
    [0, 0],
    [5, 0],
    [[[2, 1], [2, -0.5]]],
    [1],
    0,
    1,
)

problem1 = problemParameters(
    [-1, 1],
    [5, 0],
    [
        [[2, 1], [2, 2], [1, 2], [0.5, 1.5]],
        [[4, 1], [4, 0], [3, 0], [2.5, 1]],
        [[0, 0], [1, 0], [1, 1], [0, 1], [-0.5, 0.5]],
    ],
    [3, 3, 3],
    3,
    1,
)

problem2 = problemParameters(
    [1, 3],
    [10, 1],
    [
        [[2, 2], [3, 3], [3, 4], [2, 5]],
        [[6, 3], [7, 3], [7, 4], [6, 4]],
        [[4, 1], [5, 2], [4, 3], [3, 1.7]],
    ],
    [5, 11, 5],
    5,
    1,
)

problem3 = problemParameters(
    ([1, 2.5]),
    ([9, 2]),
    [
        [[3, 3], [4, 3], [4, 4], [3, 4]],
        [[6, 2], [7, 2], [7, 2.5], [6, 2.6]],
        [[5, 1], [3, 2], [1, -2]],
    ],
    [4, 4, 4],
    3,
    1,
)


problem4 = problemParameters(
    start=[-10, 2.5],
    goal=[30, 2],
    obstacles=[
        [[-3, 0], [-2, 10], [-1, 0.1], [-2, -10.6]],
        [[0, 2], [1, 3], [2, 2.8], [1, 1]],
        [[3, 3], [4, 4], [5, 3.5], [4, 2.8]],
        [[6, 2], [7, 2.5], [7, 3.7], [6, 3]],
        [[8, 1.89], [9, 2.4], [10, 1], [9, 0]],
        [[11, 4], [12, 5], [13, 4], [12, 3.8]],
        [[14, 2], [15, 3.9], [16, 2.7], [15, 1.7]],
        [[17, 0.2], [18, 10], [19, 0.8], [18, -10]],
        [[20, 3.33], [21, 4], [22, 3], [21, 2]],
        [[23, 1], [24, 2.45], [25, 1], [24, 0]],
    ],
    costs=[4, 4, 4, 4, 4, 4, 4, 4, 4, 4],
    budget=8,
    epsilon=1,
)


#######Generate problems


def generate_convex_obstacle(num_vertices, x, y):
    # Generate random vertices within a radius of 5
    vertices = []
    angle = 0
    radius = 1
    for _ in range(num_vertices):
        angle = np.random.uniform(angle, 2 * np.pi)
        radius = np.random.uniform(radius, 20)
        newX = radius * np.cos(angle) + x
        newY = radius * np.sin(angle) + y
        vertices.append([newX, newY])
    return vertices


def generate_problem(
    num_obstacles, min_vertices_per_obstacle, max_vertices_per_obstacle
):
    start = (-100, -100)
    goal = (100, 100)
    obstacles = []
    costs = []
    budget = 5
    epsilon = 1

    vertexList = []
    x = 0
    y = 0
    for _ in range(num_obstacles):
        num_vertices = np.random.randint(
            min_vertices_per_obstacle, max_vertices_per_obstacle
        )

        obstacle = generate_convex_obstacle(num_vertices, x, y)
        obstacles.append(obstacle)
        vertexList.append((x, y))

        x1 = np.random.uniform(-80, 80)
        y1 = np.random.uniform(-80, 80)
        flag = True
        while flag:
            flag = False
            for point in vertexList:
                if distance((x1, y1), point) < 40:
                    x1 = np.random.uniform(-80, 80)
                    y1 = np.random.uniform(-80, 80)
                    flag = True
                    break
        x = x1
        y = y1

        costs.append(np.random.uniform(0, 15))
    print(vertexList)
    return problemParameters(start, goal, obstacles, costs, budget, epsilon)


def plotPointsAndObstacles(start, goal, obstacles, shortestPath=None):
    """Plot only the points and obstacles"""
    plt.figure()

    # Plot obstacles
    for obs in obstacles:
        plt.fill(*zip(*obs), color="gray", alpha=0.5)
        for vertex in obs:
            plt.plot(*vertex, "ko")  # Plot obstacle vertices

    # Plot start and goal
    plt.plot(*start, "ro", label="Start")
    plt.plot(*goal, "bo", label="Goal")

    if shortestPath:
        path_array = np.array(shortestPath)
        plt.plot(path_array[:, 0], path_array[:, 1], "r-")

    # Set aspect ratio and display
    plt.gca().set_aspect("equal", adjustable="box")
    plt.legend()
    plt.show()


# problemRandom = generate_problem(20, 3, 5)

# plotPointsAndObstacles(problemRandom.start, problemRandom.goal, problemRandom.obstacles)
