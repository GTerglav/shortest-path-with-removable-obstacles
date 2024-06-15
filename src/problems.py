import math
import json
import os
import pickle
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


problem0 = problemParameters(
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
        [[6, 2], [7, 2], [7, 3], [6, 3]],
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

problemError = problemParameters(
    start=[0, 0],
    goal=[7, 0],
    obstacles=[
        [[1, 20], [1, -20], [2, -20], [2, 20]],
        [[3, 0], [3.5, 20], [4, 0], [3.5, -20]],
        [[6, 20], [6, -20], [5, -20], [5, 20]],
    ],
    costs=[3.5, 8, 3.5],
    budget=6,
    epsilon=0.5,
)

problemError2 = problemParameters(
    start=[0, 0],
    goal=[4, 0],
    obstacles=[
        [[1, 0], [0, -1], [1.5, -1]],
        [[2, -1], [1.5, 3], [3, 3]],
        [[3, 0], [2.1, -1], [3.5, -1]],
    ],
    costs=[3.5, 8, 3.5],
    budget=8,
    epsilon=0.5,
)

problemError3 = problemParameters(
    start=[0, 0],
    goal=[4, 0],
    obstacles=[
        [[1, 0], [2, -1], [2, 1]],
        [[3, 0], [2.1, -1], [2.1, 1]],
    ],
    costs=[3, 3],
    budget=3,
    epsilon=0.5,
)


#######Generate problems


def generateConvexObstacle(numVertices, x, y):
    # Generate random vertices within a radius of 5
    vertices = []
    angle = 0
    radius = 1
    for _ in range(numVertices):
        angle = np.random.uniform(angle, 2 * np.pi)
        radius = np.random.uniform(radius, 20)
        newX = radius * np.cos(angle) + x
        newY = radius * np.sin(angle) + y
        vertices.append([newX, newY])
    return vertices


def generateProblemJson(
    filename,
    numObstacles,
    minVerticesPerObstacle,
    maxVerticesPerObstacle,
    gridSize,
    obstacleDiameter,
    budget,
    epsilon,
):
    start = (-gridSize, -gridSize)
    goal = (gridSize, gridSize)
    obstacles = []
    costs = []

    vertexList = []
    x = 0
    y = 0
    for _ in range(numObstacles):
        numVertices = np.random.randint(minVerticesPerObstacle, maxVerticesPerObstacle)

        obstacle = generateConvexObstacle(numVertices, x, y)
        obstacles.append(obstacle)
        vertexList.append((x, y))

        x1 = np.random.uniform(
            -(gridSize - obstacleDiameter / 2), (gridSize - obstacleDiameter / 2)
        )
        y1 = np.random.uniform(
            -(gridSize - obstacleDiameter / 2), (gridSize - obstacleDiameter / 2)
        )
        flag = True
        while flag:
            flag = False
            for point in vertexList:
                if distance((x1, y1), point) < obstacleDiameter:
                    x1 = np.random.uniform(
                        -(gridSize - obstacleDiameter / 2),
                        (gridSize - obstacleDiameter / 2),
                    )
                    y1 = np.random.uniform(
                        -(gridSize - obstacleDiameter / 2),
                        (gridSize - obstacleDiameter / 2),
                    )
                    flag = True
                    break
        x = x1
        y = y1

        costs.append(np.random.uniform(0, budget))

    problem = {
        "start": start,
        "goal": goal,
        "obstacles": obstacles,
        "costs": costs,
        "budget": budget,
        "epsilon": epsilon,
    }
    jsonData = json.dumps(problem, indent=4)

    with open(filename, "w") as jsonFile:
        jsonFile.write(jsonData)


def generateProblemPkl(
    filename,
    numObstacles,
    minVerticesPerObstacle,
    maxVerticesPerObstacle,
    gridSize,
    obstacleDiameter,
    budget,
    epsilon,
):
    start = (-gridSize, -gridSize)
    goal = (gridSize, gridSize)
    obstacles = []
    costs = []

    vertexList = []
    x = 0
    y = 0
    for _ in range(numObstacles):
        numVertices = np.random.randint(minVerticesPerObstacle, maxVerticesPerObstacle)

        obstacle = generateConvexObstacle(numVertices, x, y)
        obstacles.append(obstacle)
        vertexList.append((x, y))

        x1 = np.random.uniform(
            -(gridSize - obstacleDiameter / 2), (gridSize - obstacleDiameter / 2)
        )
        y1 = np.random.uniform(
            -(gridSize - obstacleDiameter / 2), (gridSize - obstacleDiameter / 2)
        )
        flag = True
        while flag:
            flag = False
            for point in vertexList:
                if distance((x1, y1), point) < obstacleDiameter:
                    x1 = np.random.uniform(
                        -(gridSize - obstacleDiameter / 2),
                        (gridSize - obstacleDiameter / 2),
                    )
                    y1 = np.random.uniform(
                        -(gridSize - obstacleDiameter / 2),
                        (gridSize - obstacleDiameter / 2),
                    )
                    flag = True
                    break
        x = x1
        y = y1

        costs.append(np.random.uniform(0, budget))
    # return problemParameters(start, goal, obstacles, costs, budget, epsilon)
    problem = problemParameters(start, goal, obstacles, costs, budget, epsilon)
    with open(filename, "wb") as f:
        pickle.dump(problem, f)


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


def loadProblemPickle(filename):
    folderPath = "src\\generatedProblems"
    filePath = os.path.join(folderPath, filename)
    with open(filePath, "rb") as f:
        problem = pickle.load(f)
    return problem


# pklProblem1 = loadProblemPickle("problem40.pkl")  # n ~= 40
# pklProblem2 = loadProblemPickle("problem200.pkl")  # n ~= 200

# generateProblemPkl("problem1000.pkl", 250, 3, 5, 1000, 40, 5, 1)


# plotPointsAndObstacles(problemRandom.start, problemRandom.goal, problemRandom.obstacles)
