import math
import numpy as np
import matplotlib.pyplot as plt
from dijkstra import dijkstra
import problems


class Graph:
    def __init__(self):
        self.vertices = {}

    def addVertex(self, index):
        self.vertices[index] = []

    def addEdge(self, start, end, cost, length):
        self.vertices.setdefault(start, []).append((end, cost, length))


def distance(p1, p2):
    return math.sqrt(abs(p1[0] - p2[0]) ** 2 + abs(p1[1] - p2[1]) ** 2)


def intersect(p1, p2, p3, p4):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4

    denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

    # If lines are parallel or coincident, they don't intersect
    if denominator == 0:
        return False

    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denominator
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denominator

    # If intersection point is within both line segments, return it
    if 0 < t < 1 and 0 < u < 1:
        return True
    return False


def visibilityGraph(start, goal, obstacles, costs, budget):
    # Construct visibility graph
    points = np.vstack((start, goal, np.vstack(obstacles)))
    graph = Graph()
    for i, p1 in enumerate(points):
        graph.addVertex(i)
        for j, p2 in enumerate(points):
            if i != j:
                totalCost = 0
                for l, obs in enumerate(obstacles):
                    # If the line segment doesn't intersect but p1 and p2 belong to the same obstacle segment
                    if np.any(np.all(obs == p1, axis=1)) and np.any(
                        np.all(obs == p2, axis=1)
                    ):
                        p1_index = np.where(np.all(obs == p1, axis=1))[0][0]
                        p2_index = np.where(np.all(obs == p2, axis=1))[0][0]
                        if abs(p1_index - p2_index) not in [1, len(obs) - 1]:
                            totalCost += costs[l]
                    else:
                        # # First check the n,0 line then iterate 0,1->1,2 and so on
                        if intersect(p1, p2, obs[0], obs[-1]):
                            totalCost += costs[l]
                        else:
                            # Check for intersection with individual edges of the obstacle
                            for k in range(len(obs) - 1):
                                if intersect(p1, p2, obs[k], obs[k + 1]):
                                    totalCost += costs[l]
                                    break

                if totalCost <= budget:
                    graph.addEdge(i, j, totalCost, distance(p1, p2))
    return graph


def createCopiesOfGraph(graph, budget, epsilon):
    newGraph = Graph()
    for i, neighbors in graph.vertices.items():
        numCopies = math.ceil(budget / epsilon) + 1
        for k in range(numCopies):
            new_i = (i * numCopies) + k
            for j, cost, length in neighbors:
                new_k = math.ceil(k + cost)  # index of j with which we want to connect
                if new_k <= budget:
                    new_j = (
                        j
                    ) * numCopies + new_k  # so thid vertex is indexed as j_k' and we connect it to i_k
                    newGraph.addEdge(new_i, new_j, 0, length)
    newGraph.addVertex(-1)
    newGraph.addVertex(-2)

    # We want to create a new start (indexed as -1) and a new sink (indexed as -2) and connect them to all of their copies.
    for i in range(numCopies):
        newGraph.addEdge(-1, i, 0, 0)
        newGraph.addEdge(i, -1, 0, 0)
        newGraph.addEdge(numCopies + i, -2, 0, 0)
        newGraph.addEdge(-2, numCopies + i, 0, 0)
    return newGraph


def plotVisibilityGraph(start, goal, obstacles, graph, shortestPath=None):
    """Plot the visibility graph"""
    plt.figure()

    # Plot obstacles
    for obs in obstacles:
        plt.fill(*zip(*obs), color="gray", alpha=0.5)
        for vertex in obs:
            plt.plot(*vertex, "ko")  # Plot obstacle vertices

    # Plot edges
    for start_vertex, edges in graph.vertices.items():
        p1 = np.vstack((start, goal, np.vstack(obstacles)))[start_vertex]
        for end_vertex, _, _ in edges:
            p2 = np.vstack((start, goal, np.vstack(obstacles)))[end_vertex]
            plt.plot([p1[0], p2[0]], [p1[1], p2[1]], "b-")

    # Plot start and goal
    plt.plot(*zip(*np.vstack((start, goal))), "ro")

    # Plot shortest path if provided
    if shortestPath:
        path_array = np.array(shortestPath)
        plt.plot(path_array[:, 0], path_array[:, 1], "r-")

    # Set aspect ratio and display
    plt.gca().set_aspect("equal", adjustable="box")
    plt.show()


############################## EXECUTION ##################################


def printGraph(graph):
    for vertex, neighbors in graph.vertices.items():
        print(f"Vertex {vertex}: Neighbors {neighbors}")


def convertPathToCoordinates(path, obstacles, start, goal, budget, epsilon):
    result = ()
    points = np.vstack((start, goal, np.vstack(obstacles)))
    numCopies = math.ceil(budget / epsilon) + 1
    for vertex in path:
        if vertex not in [-1, -2]:
            pointsIndex = vertex // numCopies
            result += (points[pointsIndex],)
    return tuple(map(tuple, result))


def main(problem):
    start = problem.start
    goal = problem.goal
    obstacles = problem.obstacles
    costs = problem.costs
    budget = problem.budget
    epsilon = problem.epsilon

    graph = visibilityGraph(start, goal, obstacles, costs, budget)
    copiedGraph = createCopiesOfGraph(graph, budget, epsilon)

    start_vertex = -1
    targetVertex = -2
    shortestPath = dijkstra(copiedGraph, start_vertex, targetVertex)

    if shortestPath:
        nicePath = convertPathToCoordinates(
            shortestPath, obstacles, start, goal, budget, epsilon
        )
        print("Shortest path:", nicePath)

    plotVisibilityGraph(start, goal, obstacles, graph, nicePath)


if __name__ == "__main__":
    main(problems.problem2)
