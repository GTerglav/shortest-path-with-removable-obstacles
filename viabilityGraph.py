import math
import time
import numpy as np
from dijkstra import dijkstra
from helper import distance, intersect2, plotViabilityGraph, plotPointsAndObstacles
import problems
from problems import problemParameters

# Naive construction of viability graph O(n^3)
# For now includes a bug that add an edge that shouldnt be there if the 3 points are colinear


class Graph:
    def __init__(self):
        self.vertices = {}

    def addVertex(self, index):
        self.vertices[index] = []

    def addEdge(self, start, end, cost, length):
        self.vertices.setdefault(start, []).append((end, cost, length))

# Construct viability graph
def viabilityGraph(start, goal, obstacles, costs, budget):
    points = np.vstack((start, goal, np.vstack(obstacles)))
    graph = Graph()
    for i, p1 in enumerate(points):
        graph.addVertex(i)
        for j, p2 in enumerate(points):
            if i != j:
                totalCost = 0
                for l, obs in enumerate(obstacles):
                    # If the line segment doesn't intersect2 but p1 and p2 belong to the same obstacle segment
                    if np.any(np.all(obs == p1, axis=1)) and np.any(
                        np.all(obs == p2, axis=1)
                    ):
                        p1_index = np.where(np.all(obs == p1, axis=1))[0][0]
                        p2_index = np.where(np.all(obs == p2, axis=1))[0][0]
                        if abs(p1_index - p2_index) not in [1, len(obs) - 1]:
                            totalCost += costs[l]
                    else:
                        # # First check the n,0 line then iterate 0,1->1,2 and so on
                        if intersect2(p1, p2, obs[0], obs[-1]):
                            totalCost += costs[l]
                        else:
                            # Check for intersect2ion with individual edges of the obstacle
                            for k in range(len(obs) - 1):
                                if intersect2(p1, p2, obs[k], obs[k + 1]):
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
                new_k = max(
                    k, math.floor((k * epsilon + cost) / epsilon)
                )  # index of j with which we want to connect
                if new_k <= numCopies - 1:
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


def main(problem, epsilon=None):
    start = problem.start
    goal = problem.goal
    obstacles = problem.obstacles
    costs = problem.costs
    budget = problem.budget
    if epsilon is None:
        epsilon = problem.epsilon

    startTime = time.time()
    graph = viabilityGraph(start, goal, obstacles, costs, budget)
    endTime = time.time()
    print(f"Viability graph construction time {endTime - startTime} seconds")

    startTime = time.time()
    copiedGraph = createCopiesOfGraph(graph, budget, epsilon)
    start_vertex = -1
    targetVertex = -2
    shortestPath = dijkstra(copiedGraph, start_vertex, targetVertex)
    endTime = time.time()
    print(f"Copying graph and dijsktra time {endTime - startTime} seconds")

    if shortestPath:
        nicePath = convertPathToCoordinates(
            shortestPath, obstacles, start, goal, budget, epsilon
        )
        # plotPointsAndObstacles(start, goal, obstacles, nicePath)
        print(f"Shortest path from {start} to {goal} is {nicePath}")

    # plotViabilityGraph(start, goal, obstacles, graph, nicePath)


pklProblem40 = problems.loadProblemPickle("problem40.pkl")
pklProblem200 = problems.loadProblemPickle("problem200.pkl")
# n = 180, time = 36s
pklProblem400 = problems.loadProblemPickle("problem400.pkl")
# n = 360, time = 306s ->
pklProblem1000 = problems.loadProblemPickle("problem1000.pkl")
if __name__ == "__main__":
    startTime2 = time.time()
    main(pklProblem40)
    endTime2 = time.time()
    print(f"Execution time {endTime2 - startTime2} seconds")

    ######## Code for measuring time for different epsilons #########
    # epsilons = [1, 0.5, 0.25, 0.1, 0.01, 0.001, 0.0001, 0.00001]
    # for epsilon in epsilons:
    #     start_time = time.time()
    #     main(problems.problem2, epsilon)  # Use the problem you defined
    #     end_time = time.time()
    #     print(f"Epsilon {epsilon}: Execution time {end_time - start_time} seconds")
