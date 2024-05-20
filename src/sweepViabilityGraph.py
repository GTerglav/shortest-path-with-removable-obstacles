import math
import time

from matplotlib import pyplot as plt
import numpy as np
from AVLTree import AVLTree, TreeNode

from dijkstra import dijkstra
import helper

from problems import problemParameters
import problems


class Graph:
    def __init__(self):
        self.vertices = {}

    def addVertex(self, index):
        self.vertices[tuple(index)] = []

    def addEdge(self, start, end, cost, length):
        start_tuple = tuple(start)
        end_tuple = tuple(end)
        self.vertices.setdefault(start_tuple, []).append((end_tuple, cost, length))


############################### Viability algorithm #############################################


# Returns cost of path from p to w. wMinusOne is vertex before w in angular ordering
def viable(p, w, wMinusOne, costToWMinusOne, root, obstacles, costs):

    # check if p and w are in same obstacle segment
    sameObstacleResult = helper.inSameObstacleSegment(
        p, w, obstacles, costs
    )  # Runs O(n) thats why the whole thing is O(n^3)
    if sameObstacleResult is not None:
        obstacle, cost = sameObstacleResult
        if helper.areNeighboursInObstacle(p, w, obstacle):
            return 0
        else:
            return cost

    # If w and w_-1 are not colinear then just sum up all the obstacles on the path
    elif (wMinusOne is None) or (helper.ccw(p, w, wMinusOne)) != 0:
        if root is None:
            return 0
        return helper.costOfSegment(w, p, root, costs)

    # If they are colinear then we know the cost to w_-1 just need the cost from w_-1 to w
    else:
        totalCost = costToWMinusOne
        obstacleResult = helper.inSameObstacleSegment(w, wMinusOne, obstacles, costs)
        if obstacleResult is not None:
            obstacle, cost = obstacleResult
            if helper.areNeighboursInObstacle(w, wMinusOne, obstacle):
                return totalCost
            else:
                return totalCost + cost
        else:
            return totalCost + helper.costOfSegment(w, wMinusOne, root, costs)


# Returns viable vertices from v and costs of those direct paths
def viableVerticesFromV(v, points, obstacles, costs, budget):
    # Sort vertices accoring to clockwise angle from v and distance from v
    sortedVertices = sorted(points, key=lambda x: helper.compareAngle(x, v))

    T = AVLTree()
    root = None

    # Insert edges that intersect into tree
    for j, obstacle in enumerate(obstacles):
        for i in range(len(obstacle)):
            start = obstacle[i]
            end = obstacle[(i + 1) % len(obstacle)]
            t = helper.intersectLine(v, (v[0] + 1, v[1]), start, end)
            if t < math.inf:
                # So our tree stores the following info about edges:
                # start, end, distance from v,cost of whole obstacle and obstalce name
                root = T.insert(root, (start, end, t, costs[j], j))

    W = set()
    wMinusOne = None
    costToWMinusOne = 0
    for w in sortedVertices:
        if not np.array_equal(v, w):
            costOfPathToW = viable(
                v, w, wMinusOne, costToWMinusOne, root, obstacles, costs
            )
            # print(v, w, costOfPathToW)
            if costOfPathToW <= budget:
                W.add((tuple(w), costOfPathToW))

            # But all edges that include w should be on the counter clockwise side so I can just delete them all
            root = T.deleteByVertex(root, w)

            # need to insert into T edges that include w and lie on the clockwise side of the line
            for k, obstacle in enumerate(obstacles):
                for i in range(len(obstacle)):
                    start = obstacle[i]
                    end = obstacle[(i + 1) % len(obstacle)]
                    if (helper.ccw(v, w, start) <= 0 and np.array_equal(end, w)) or (
                        helper.ccw(v, w, end) <= 0 and np.array_equal(start, w)
                    ):
                        root = T.insert(
                            root,
                            (
                                start,
                                end,
                                # I dont want two of the same key for avl trees
                                # Maybe should sort them by clockwise angle??? I dont know this is wrong
                                (helper.distance(v, start) + helper.distance(v, end))
                                / 2,
                                costs[k],
                                k,
                            ),
                        )
            # we need to store w_-1
            wMinusOne = w
            costToWMinusOne = costOfPathToW
    return W


# Returns viability graph
def viabilityGraph(start, goal, obstacles, costs, budget):
    points = np.vstack((start, goal, np.vstack(obstacles)))
    graph = Graph()
    for p1 in points:
        graph.addVertex(p1)
        viable = viableVerticesFromV(p1, points, obstacles, costs, budget)
        for p2, totalCost in viable:
            graph.addEdge(p1, p2, totalCost, helper.distance(p1, p2))

    return graph


def createCopiesOfGraph(graph, start, goal, budget, epsilon):
    newGraph = Graph()

    for vertex, neighbors in graph.vertices.items():
        numCopies = math.ceil(budget / epsilon) + 1
        for k in range(numCopies):
            newVertex = (vertex[0], vertex[1], k)
            for neighbor, cost, length in neighbors:
                new_k = max(k, math.floor((k * epsilon + cost) / epsilon))
                if new_k <= numCopies - 1:
                    newNeighbor = (neighbor[0], neighbor[1], new_k)
                    newGraph.addEdge(newVertex, newNeighbor, 0, length)

    # add start and sink and connect them to copies
    newStart = (-1, -1, -1)
    newSink = (-2, -2, -2)
    newGraph.addVertex(newStart)
    newGraph.addVertex(newSink)
    # Create new start (indexed as -1) and sink (indexed as -2) vertices and connect them to all of their copies
    for i in range(numCopies):
        newGraph.addEdge(newStart, (start[0], start[1], i), 0, 0)
        newGraph.addEdge((start[0], start[1], i), newStart, 0, 0)
        newGraph.addEdge((goal[0], goal[1], i), newSink, 0, 0)
        newGraph.addEdge(newSink, (goal[0], goal[1], i), 0, 0)
    return newGraph


# Plots the viablity graph
def plotGraph(graph, start, goal, obstacles, costs, budget, epsilon):
    # Plot edges
    for vertex, edges in graph.vertices.items():
        for edge in edges:
            startPoint = np.array(vertex)
            endPoint, _, _ = edge
            endPoint = np.array(endPoint)
            plt.plot(
                [startPoint[0], endPoint[0]],
                [startPoint[1], endPoint[1]],
                "b-",
                linewidth=0.1,
            )

    # Plot obstacles and their costs
    for i, obstacle in enumerate(obstacles):
        xCoords = [point[0] for point in obstacle]
        yCoords = [point[1] for point in obstacle]
        plt.fill(xCoords, yCoords, "gray", alpha=0.5)
        for vertex in obstacle:
            plt.plot(*vertex, "ko")  # Plot obstacle vertices
        # Compute centroid of the obstacle for placing the cost text
        centroid_x = np.mean(xCoords)
        centroid_y = np.mean(yCoords)
        plt.text(
            centroid_x,
            centroid_y,
            f"{costs[i]}",
            color="black",
            fontsize=13,
            ha="center",
        )

    # Plot start and end vertices
    plt.scatter(start[0], start[1], color="green", label="Start")
    plt.scatter(goal[0], goal[1], color="red", label="Goal")

    # Display budget and epsilon values above the graph
    plt.text(
        0.5,
        1.05,
        f"Budget: {budget}, Epsilon: {epsilon}",
        transform=plt.gca().transAxes,
        ha="center",
        fontsize=14,
    )

    plt.xlabel("X")
    plt.ylabel("Y")
    # plt.title("Graph with Obstacles")
    # plt.legend()
    plt.show()


# Returns shortest path for given problem
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

    copiedGraph = createCopiesOfGraph(graph, start, goal, budget, epsilon)

    startVertex = (-1, -1, -1)
    targetVertex = (-2, -2, -2)
    shortestPath = dijkstra(copiedGraph, startVertex, targetVertex)

    if shortestPath:
        nicePath = shortestPath[1:-1]

        # Plot the problem, then problem w/ shortest path, then viability graph
        helper.plotProblem(start, goal, obstacles, budget, costs)
        helper.plotPointsAndObstaclesSweep(
            start, goal, obstacles, budget, costs, epsilon, nicePath
        )
        plotGraph(graph, start, goal, obstacles, costs, budget, epsilon)
        print(f"Shortest path from {start} to {goal} is {nicePath}")
        return nicePath
    else:
        return []


pklProblem40 = problems.loadProblemPickle("problem40.pkl")
# n ~= 40, time = 1s
pklProblem200 = problems.loadProblemPickle("problem200.pkl")
# n = 180, time = 60s
pklProblem400 = problems.loadProblemPickle("problem400.pkl")
# n = 360, time = 480s which is 2^3 * time more than before
pklProblem1000 = problems.loadProblemPickle("problem1000.pkl")
# n = 864, time = 6666s ~= (864/360)^3 * 480 = 6200
# All of the running time is just for the construction of viability graph

## conclusion n^3 checks out

if __name__ == "__main__":
    startTime = time.time()
    main(problems.problemError, 1)
    endTime = time.time()
    print(f"Execution time {endTime - startTime} seconds")

    ######## Code for measuring time for different epsilons #########
    # epsilons = [1, 0.5, 0.25, 0.1, 0.01, 0.001, 0.0001, 0.00001]
    # for epsilon in epsilons:
    #     startTime = time.time()
    #     main(problems.problem4, epsilon)
    #     endTime = time.time()
    #     print(f"Epsilon {epsilon}: Execution time {endTime - startTime} seconds")
