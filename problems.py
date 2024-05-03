import numpy as np
import random


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
        [[4, 1], [4, 1], [4, 0], [3, 0], [2.5, 1]],
        [[0, 0], [1, 0], [1, 1], [0, 1], [-0.5, 0.5]],
    ],
    [3, 3, 3],
    1,
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
    [8, 5, 10],
    7,
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
    4,
    1,
)


def generate_random_problem(
    num_obstacles, min_vertices_per_obstacle, max_vertices_per_obstacle
):
    start = np.random.uniform(-10, 10, size=(2,))
    goal = np.random.uniform(-10, 10, size=(2,))
    obstacles = []
    total_vertices = 0

    for _ in range(num_obstacles):
        num_vertices = random.randint(
            min_vertices_per_obstacle, max_vertices_per_obstacle
        )
        total_vertices += num_vertices
        obstacle = []
        for _ in range(num_vertices):
            vertex = np.random.uniform(-10, 10, size=(2,))
            obstacle.append(vertex)
        obstacles.append(obstacle)

    costs = [random.uniform(0.5, 5) for _ in range(num_obstacles)]
    budget = random.randint(50, 200)
    epsilon = random.uniform(0.5, 2)

    return problemParameters(start, goal, obstacles, costs, budget, epsilon)


# Example usage:
problem = generate_random_problem(
    num_obstacles=3, min_vertices_per_obstacle=5, max_vertices_per_obstacle=10
)
