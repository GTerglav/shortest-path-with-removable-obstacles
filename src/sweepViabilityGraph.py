import math
import time
from matplotlib import pyplot as plt
import numpy as np
from AVLTree import AVLTree
from dijkstra import dijkstra
from problems import problemParameters
import problems


# Want this class so i know to what obstacle the points belong
# Previously had to search through all obstacles to find this
# Maybe could add some other usefull info
class Point:
    def __init__(self, x, y, obstacleIndex, obstacleName):
        self.x = x
        self.y = y
        self.obstacleIndex = obstacleIndex  # each point in obstacle has different index
        self.obstacleName = obstacleName  # name of obstacle the point belongs


class Graph:
    def __init__(self):
        self.vertices = {}

    def addVertex(self, index):
        self.vertices[index] = []

    def addEdge(self, start, end, cost, length):
        self.vertices.setdefault(start, []).append((end, cost, length))


############################## Helper functions modified to suit Point class ############


def plotPointsAndObstaclesSweep(
    start, goal, obstacles, budget, costs, epsilon, shortestPath=None
):
    plt.figure()

    # Plot obstacles
    for i, obs in enumerate(obstacles):
        xCoords = [point.x for point in obs]
        yCoords = [point.y for point in obs]
        plt.fill(xCoords, yCoords, color="gray", alpha=0.5)
        for vertex in obs:
            plt.plot(vertex.x, vertex.y, "ko")  # Plot obstacle vertices

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

    # Plot start and goal
    plt.plot(start.x, start.y, "ro", label="Start")
    plt.plot(goal.x, goal.y, "bo", label="Goal")

    if shortestPath:
        path = [(x, y) for x, y, _ in shortestPath]
        pathArray = np.array(path)
        plt.plot(pathArray[:, 0], pathArray[:, 1], "r-")

    plt.text(
        0.5,
        1.05,
        f"Budget: {budget}, Epsilon: {epsilon}",
        transform=plt.gca().transAxes,
        ha="center",
        fontsize=14,
    )

    # Set aspect ratio and display
    # plt.gca().set_aspect("equal", adjustable="box")
    # plt.legend()
    plt.xlabel("X")
    plt.ylabel("Y")
    # plt.title("Graph with Obstacles")
    # plt.grid(True)
    plt.show()


def intersect(p1, p2, p3, p4):
    x1, y1 = p1.x, p1.y
    x2, y2 = p2.x, p2.y
    x3, y3 = p3.x, p3.y
    x4, y4 = p4.x, p4.y

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


def ccw(p1, p2, p3):
    if p3 is None:
        return 1
    else:
        return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)


def costOfSegment(v, w, root, costs):
    if root is None:
        return 0

    totalCost = 0
    # want to keep track of costs
    costDict = {}

    # Traverse the AVL tree to find edges
    stack = [root]
    while stack:
        node = stack.pop()
        if node is not None:
            start, end, _, cost, obstacleName = node.key
            if intersect(v, w, start, end):
                totalCost += cost
                if obstacleName in costDict:
                    costDict.pop(obstacleName)
                else:
                    costDict[obstacleName] = True
            stack.append(node.left)
            stack.append(node.right)

    # If the cost is counted twice we have to divide by two if its counted once then its full cost
    if len(costDict) == 0:
        totalCost /= 2
    else:
        singleCosts = 0
        for obstacleName in costDict.keys():
            singleCosts += costs[obstacleName]
        totalCost = (totalCost - singleCosts) / 2 + singleCosts
    return totalCost


def distance(p1, p2):
    return math.sqrt(abs(p1.x - p2.x) ** 2 + abs(p1.y - p2.y) ** 2)


def compareAngle(v, p):
    angle = math.atan2(p.y - v.y, p.x - v.x)
    dist = distance(v, p)
    return (-angle, dist)


# Checks if halfline v p intersects obstacle segment start end
def intersectLine(v, p, start, end):
    # Calculate the direction vectors of the edge (start, end) and the line (v, p)
    b0 = end.x - start.x
    b1 = end.y - start.y
    a0 = p.x - v.x
    a1 = p.y - v.y

    determinant = b0 * a1 - b1 * a0

    # Check if the determinant is close to zero, indicating collinearity
    if abs(determinant) < 1e-10:
        if ccw(v, p, start) == 0:
            return min(distance(v, start), distance(v, end))
        else:
            return math.inf
    else:
        # Calculate the parameters t and u for the intersection point
        # t = ((v[0] - start[0]) * b1 - (v[1] - start[1]) * b0) / determinant
        u = ((v.x - start.x) * a1 - (v.y - start.y) * a0) / determinant
        t = (b1 * (v.x - start.x) + b0 * (start.y - v.y)) / determinant

        # Check if the intersection point lies within the line segment vp
        if t >= 0 and 0 < u < 1:
            return t
        else:
            return math.inf


############################### Viability algorithm #############################################


# Returns cost of path from p to w. wMinusOne is vertex before w in angular ordering
def viable(p, w, wMinusOne, costToWMinusOne, root, obstacles, costs):
    i = p.obstacleIndex  # order of i in obstacle
    j = w.obstacleIndex
    n = len(
        obstacles[p.obstacleName]
    )  # amount of vertices in obstacle that p belongs to
    # check if p and w are in same obstacle segment
    if p.obstacleName == w.obstacleName:
        if (i + 1) % n == j % n or (i - 1) % n == j % n:
            return 0
        else:
            return costs[p.obstacleName]

    # If w and w_-1 are not colinear then just sum up all the obstacles on the path
    elif (wMinusOne is None) or (ccw(p, w, wMinusOne)) != 0:
        if root is None:
            return 0
        return costOfSegment(w, p, root, costs)

    # If they are colinear then we know the cost to w_-1 just need the cost from w_-1 to w
    else:
        totalCost = costToWMinusOne
        if p.obstacleName == w.obstacleName:
            if (i + 1) % n == j % n or (i - 1) % n == j % n:
                return totalCost
            else:
                return totalCost + costs[p.obstacleName]
        else:
            return totalCost + costOfSegment(w, wMinusOne, root, costs)


# Returns viable vertices from v and costs of those direct paths
def viableVerticesFromV(v, points, obstacles, costs, budget):
    # Sort vertices accoring to clockwise angle from v and distance from v
    sortedVertices = sorted(points, key=lambda x: compareAngle(x, v))

    T = AVLTree()
    root = None

    # Insert edges that intersect into tree
    for j, obstacle in enumerate(obstacles):
        for i in range(len(obstacle)):
            start = obstacle[i]
            end = obstacle[(i + 1) % len(obstacle)]
            vPlus = Point(v.x + 1, v.y, v.obstacleIndex, v.obstacleName)
            t = intersectLine(v, vPlus, start, end)
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
                W.add((w, costOfPathToW))

            # But all edges that include w should be on the counter clockwise side so I can just delete them all
            root = T.deleteByVertex(root, w)

            # need to insert into T edges that include w and lie on the clockwise side of the line
            for k, obstacle in enumerate(obstacles):
                for i in range(len(obstacle)):
                    start = obstacle[i]
                    end = obstacle[(i + 1) % len(obstacle)]
                    if (ccw(v, w, start) <= 0 and np.array_equal(end, w)) or (
                        ccw(v, w, end) <= 0 and np.array_equal(start, w)
                    ):
                        root = T.insert(
                            root,
                            (
                                start,
                                end,
                                # I dont want two of the same key for avl trees
                                # Maybe should sort them by clockwise angle??? I dont know this is wrong
                                (distance(v, start) + distance(v, end)) / 2,
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
    # points = np.vstack((start, goal, np.vstack(obstacles)))
    points = [start, goal]
    for obstacle in obstacles:
        for point in obstacle:
            points.append(point)
    graph = Graph()
    for p1 in points:
        graph.addVertex(p1)
        viable = viableVerticesFromV(p1, points, obstacles, costs, budget)
        for p2, totalCost in viable:
            graph.addEdge(p1, p2, totalCost, distance(p1, p2))

    return graph


def createCopiesOfGraph(graph, start, goal, budget, epsilon):
    newGraph = Graph()

    for vertex, neighbors in graph.vertices.items():
        numCopies = math.ceil(budget / epsilon) + 1
        for k in range(numCopies):
            newVertex = (vertex.x, vertex.y, k)
            for neighbor, cost, length in neighbors:
                new_k = max(k, math.floor((k * epsilon + cost) / epsilon))
                if new_k <= numCopies - 1:
                    newNeighbor = (neighbor.x, neighbor.y, new_k)
                    newGraph.addEdge(newVertex, newNeighbor, 0, length)

    # add start and sink and connect them to copies
    newStart = (-1, -1, -1)
    newSink = (-2, -2, -2)
    newGraph.addVertex(newStart)
    newGraph.addVertex(newSink)
    # Create new start (indexed as -1) and sink (indexed as -2) vertices and connect them to all of their copies
    for i in range(numCopies):
        newGraph.addEdge(newStart, (start.x, start.y, i), 0, 0)
        newGraph.addEdge((start.x, start.y, i), newStart, 0, 0)
        newGraph.addEdge((goal.x, goal.y, i), newSink, 0, 0)
        newGraph.addEdge(newSink, (goal.x, goal.y, i), 0, 0)
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
                [startPoint.x, endPoint.x],
                [startPoint.y, endPoint.y],
                "b-",
                linewidth=0.1,
            )

    # Plot obstacles and their costs
    for i, obstacle in enumerate(obstacles):
        xCoords = [point.x for point in obstacle]
        yCoords = [point.y for point in obstacle]
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
    plt.scatter(start.x, start.y, color="green", label="Start")
    plt.scatter(goal.x, goal.y, color="red", label="Goal")

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
    plt.show()


# Returns shortest path for given problem
def main(problem, epsilon=None):
    start = problem.start
    goal = problem.goal
    obstaclesTemp = problem.obstacles
    costs = problem.costs
    budget = problem.budget

    # Transform the points into Point Class
    start = Point(start[0], start[1], 0, -1)
    goal = Point(goal[0], goal[1], 0, -2)
    obstacles = []
    for i, obstacle in enumerate(obstaclesTemp):
        list = []
        for j, point in enumerate(obstacle):
            p = Point(point[0], point[1], j, i)
            list.append(p)
        obstacles.append(list)

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
        # plotProblem(start, goal, obstacles, budget, costs)
        # plotPointsAndObstaclesSweep(
        #     start, goal, obstacles, budget, costs, epsilon, nicePath
        # )
        # plotGraph(graph, start, goal, obstacles, costs, budget, epsilon)
        print(f"Shortest path from {start} to {goal} is {nicePath}")
        return nicePath

    else:
        return []


pklProblem40 = problems.loadProblemPickle("problem40.pkl")
# n ~= 40, time = 0.7s
pklProblem200 = problems.loadProblemPickle("problem200.pkl")
# n = 180, time = 50s
pklProblem400 = problems.loadProblemPickle("problem400.pkl")
# n = 360, time = 393s
pklProblem1000 = problems.loadProblemPickle("problem1000.pkl")
# n = 864, time = 6666s ~= (864/360)^3 * 480 = 6200
# All of the running time is just for the construction of viability graph


if __name__ == "__main__":
    startTime = time.time()
    main(pklProblem1000)
    endTime = time.time()
    print(f"Execution time {endTime - startTime} seconds")

    ####### Code for measuring time for different epsilons #########
    # epsilons = [1, 0.5, 0.25, 0.1, 0.01, 0.001, 0.0001, 0.00001]
    # for epsilon in epsilons:
    #     startTime = time.time()
    #     main(problems.problem4, epsilon)
    #     endTime = time.time()
    #     print(f"Epsilon {epsilon}: Execution time {endTime - startTime} seconds")
