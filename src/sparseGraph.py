import math
import statistics
import time

from matplotlib import pyplot as plt
import numpy as np
from AVLTree import AVLTree, TreeNode

from dijkstra import dijkstra
import helper

from problems import problemParameters
import problems

from persistentRBTree import persistentRBTree

########### First copy all the code from sweepVisGraph to obtain the viability graph we want to sparsify


class Graph:
    def __init__(self):
        self.vertices = {}

    def addVertex(self, index):
        self.vertices[tuple(index)] = []

    def addEdge(self, start, end, cost, length):
        start_tuple = tuple(start)
        end_tuple = tuple(end)
        self.vertices.setdefault(start_tuple, []).append((end_tuple, cost, length))


def viable(p, w, wMinusOne, costToWMinusOne, root, obstacles, costs):
    # check if p and w are in same obstacle segment
    sameObstacleResult = helper.inSameObstacleSegment(p, w, obstacles, costs)
    if sameObstacleResult is not None:
        obstacle, cost = sameObstacleResult
        if helper.areNeighboursInObstacle(p, w, obstacle):
            return 0
        else:
            return cost

    elif (wMinusOne is None) or (helper.ccw(p, w, wMinusOne)) != 0:
        if root is None:
            return 0

        totalCost = 0
        count = 0
        costDict = {}

        # Traverse the AVL tree to find edges
        stack = [root]
        while stack:
            node = stack.pop()
            if node is not None:
                start, end, _, cost = node.key
                if helper.intersect2(p, w, start, end):
                    totalCost += cost
                    count += 1
                    if cost in costDict:
                        costDict.pop(cost)
                    else:
                        costDict[cost] = True
                stack.append(node.left)
                stack.append(node.right)

        if len(costDict) == 0:
            totalCost /= 2
        else:
            singleCost = sum(costDict.keys())
            totalCost = (totalCost - singleCost) / 2 + singleCost
        return totalCost

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
            return totalCost + helper.costOfSegment(w, wMinusOne, root)


def viableVerticesFromV(v, points, obstacles, costs, budget):
    # here sort vertices accoring to angle from v and distance from v
    sortedVertices = sorted(points, key=lambda x: helper.compareAngle(x, v))

    T = AVLTree()
    root = None

    for j, obstacle in enumerate(obstacles):
        for i in range(len(obstacle)):
            start = obstacle[i]
            end = obstacle[(i + 1) % len(obstacle)]
            t = helper.intersectLine(v, (v[0] + 1, v[1]), start, end)
            if t < math.inf:
                root = T.insert(root, (start, end, t, costs[j]))

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

            root = T.deleteByVertex(root, w)

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
                                (helper.distance(v, start) + helper.distance(v, end))
                                / 2,
                                costs[k],
                            ),
                        )
            wMinusOne = w
            costToWMinusOne = costOfPathToW
    return W


def viabilityGraph(start, goal, obstacles, costs, budget):
    points = np.vstack((start, goal, np.vstack(obstacles)))
    graph = Graph()
    for p1 in points:
        graph.addVertex(p1)
        viable = viableVerticesFromV(p1, points, obstacles, costs, budget)
        for p2, totalCost in viable:
            graph.addEdge(p1, p2, totalCost, helper.distance(p1, p2))

    return graph


######################## Now to sparsify the graph ###############

# Define relation and equality for addding obstacle segments to tree


# a,b line segments of form [point1,point2]
def equality(a, b):
    return sorted(a) == sorted(b)


# For vertical split lines. "Time" is on y-axis, order on x-axis.
# returns a<b
def verticalRelation(a, b):
    if a[0][1] >= a[1][1]:
        a = [a[1], a[0]]  # point with lower y-component is first
    if b[0][1] >= b[1][1]:
        b = [b[1], b[0]]

    low = a if a[0][1] <= b[0][1] else b
    high = b if a[0][1] <= b[0][1] else a

    if helper.ccw(low[0], low[1], high[0]) > 0:
        # low > high
        return b == low
    elif helper.ccw(low[0], low[1], high[0]) < 0:
        return b != low
    else:
        if helper.ccw(low[0], low[1], high[1]) > 0:
            # low > high
            return b == low
        else:
            return b != low


# For horizontal split lines. "Time" is on x-axis, order on y-axis.
def horizontalRelation(a, b):
    if a[0][0] >= a[1][0]:
        a = [a[1], a[0]]  # point with lower x-component is first
    if b[0][0] >= b[1][0]:
        b = [b[1], b[0]]

    low = a if a[0][0] <= b[0][0] else b
    high = b if a[0][0] <= b[0][0] else a

    if helper.ccw(low[0], low[1], high[0]) > 0:
        return b == high
    elif helper.ccw(low[0], low[1], high[0]) < 0:
        return b != high
    else:
        if helper.ccw(low[0], low[1], high[1]) > 0:
            return b == high
        else:
            return b != high


a = [[1, 0], [2, -1]]
b = [[1, 0], [2, 1]]

print(horizontalRelation(a, b))


# Make tree of obstacle segments for vertical split lines. "Time" is on y-axis
def makeVerticalPersistentTree(obstacles, costs, relation, equality):
    tree = persistentRBTree(relation, equality)

    # Just a list of all points with costs
    points = []
    for i, obstacle in enumerate(obstacles):
        for vertex in obstacle:
            points.append([vertex, costs[i]])

    # Sort them by y, first has lowest
    sortedPoints = sorted(points, key=lambda point: point[0][1])

    for point in sortedPoints:
        # two neighbors in obstacle
        neighbors = helper.findObstacleEdges(obstacles, point[0])
        if neighbors:
            for nh in neighbors:
                # not considering "= case" since the obstacle segments in such cases are not positive or negative
                # insert
                if nh[1] > point[0][1]:
                    key = [point[0], nh]  # keys are endpoints of segment
                    if nh[0] != point[0][0]:
                        # val is 1st vertex of edge, 2nd, cost of obstacle of edge
                        tree.insert(key, [point[0], nh, point[1]], point[0][1])
                # delete
                elif nh[1] < point[0][1]:
                    key = [nh, point[0]]
                    tree.delete(key, point[0][1])
    return tree


# Make tree of obstacle segments for horizontal split lines. "Time" is on x-axis
def makeHorizontalPersistentTree(obstacles, costs, relation, equality):
    tree = persistentRBTree(relation, equality)

    points = []
    for i, obstacle in enumerate(obstacles):
        for vertex in obstacle:
            points.append([vertex, costs[i]])

    # Sort them by x, first has lowest
    sortedPoints = sorted(points, key=lambda point: point[0][0])

    for point in sortedPoints:
        neighbors = helper.findObstacleEdges(obstacles, point[0])
        if neighbors:
            for nh in neighbors:
                if nh[0] > point[0][0]:
                    key = [point[0], nh]
                    if nh[1] != point[0][1]:
                        tree.insert(key, [point[0], nh, point[1]], point[0][0])
                elif nh[0] < point[0][0]:
                    key = [nh, point[0]]
                    tree.delete(key, point[0][0])
    return tree


# TODO
# Computes cost of edge (v,u)
def cost(v, u, obstacles, costs):
    return 0


# TODO
# Finds first positive slope segmenet that intersects (v,u).
# (v,u) is horizontal or vertical
def positiveObstacleSegment(v, u, tree):
    if v[0] == u[0]:  # (u,v) vertical
        lb = [v, v]
        hb = [u, u]
        if v[1] < u[1]:
            edges = tree.accessRange(lb, hb, v[0])
        else:
            edges = tree.accessRange(hb, lb, v[0])
    elif v[1] == u[1]:  # (u,v) horiznotal
        if v[0] < u[0]:
            edges = tree.accessRange(lb, hb, v[1])
        else:
            edges = tree.accessRange(hb, lb, v[1])

    # v is projection of u so we need the edge closest to u with pos/neg slope.
    for edge in edges:
        node, edge = edge[0], edge[1]
        cost = node.val

    return ((0, 0), (1, 1))


# TODO
# Finds first negative slope segmenet that intersects (v,u)
def negativeObstacleSegment(v, u, obstacles):
    return ((0, 0), (1, 1))


# TODO
# This new graph should start as the viability graph
# vertical == True then vertical split line otherwise horizontal
def Recurse(vertices, start, goal, obstacles, costs, budget, vertical=True):
    # This function is going to returna list if all edges and vertices that need to be added to our viability graph
    newVertices = []
    newEdges = []

    if len(vertices) <= 1:
        return [newVertices, newEdges]

    # get median x coordiante
    coordinates = []
    for vertex in vertices:
        if vertical:
            coordinates.append(vertex[0])
        else:
            coordinates.append(vertex[1])
    median = statistics.median(coordinates)

    for vertex in vertices:
        if vertical:
            projectedVertex = (median, vertex[1])
        else:
            projectedVertex = (vertex[0], median)
        cost = cost(vertex, projectedVertex, obstacles, costs)

        # a) First add the projected vertex to graph
        if cost <= budget:
            # newGraph.addVertex(projectedVertex)
            # newGraph.addEdge(vertex,projectedVertex, cost, helper.distance(vertex,projectedVertex))
            newVertices.append(projectedVertex)
            newEdges.append(
                (
                    vertex,
                    projectedVertex,
                    cost,
                    helper.distance(vertex, projectedVertex),
                )
            )

        # b),c) Check first intersecting obstacle segments
        posSegment = positiveObstacleSegment(vertex, projectedVertex, obstacles)
        negSegment = negativeObstacleSegment(vertex, projectedVertex, obstacles)
        if posSegment is not None:
            # Do stuff
            "TODO"

        if negSegment is not None:
            # again do stuff
            "TODO"

        # d) Connect adjacent "Steiner" (new) vertices
        sortedVertices = sorted(newVertices, key=lambda vertex: vertex[1])
        for i in range(len(sortedVertices) - 1):
            v = sortedVertices[i]
            u = sortedVertices[i + 1]
            cost = cost(v, u, obstacles, costs)
            if cost <= budget:
                newEdges.append(
                    (
                        v,
                        u,
                        cost,
                        helper.distance(v, u),
                    )
                )

    # e) split vertices into two groups
    # Should be named bot and top if horizontal
    leftVertices = []
    rightVertices = []
    for vertex in vertices:
        if vertical:
            if vertex[0] <= median:
                leftVertices.append(vertex)
            else:
                rightVertices.append(vertex)
        else:
            if vertex[1] <= median:
                leftVertices.append(vertex)
            else:
                rightVertices.append(vertex)

    return [
        newVertices
        + Recurse(rightVertices, start, goal, obstacles, costs, budget)[0]
        + Recurse(leftVertices, start, goal, obstacles, costs, budget)[0],
        newEdges
        + Recurse(rightVertices, start, goal, obstacles, costs, budget)[1]
        + Recurse(leftVertices, start, goal, obstacles, costs, budget)[1],
    ]


# TODO
def sparsify(graph, start, goal, obstacles, costs, budget):

    # 1. Step add new edges and vertices
    newVerticesV, newEdgesV = Recurse(
        graph, start, goal, obstacles, costs, budget, True
    )

    for vertex in newVerticesV:
        graph.addVertex(vertex)

    for beginning, end, cost, distance in newEdgesV:
        graph.addEdge(beginning, end, cost, distance)

    # 2. Step add new edges and vertices
    newVerticesH, newEdgesH = Recurse(
        graph, start, goal, obstacles, costs, budget, False
    )

    for vertex in newVerticesH:
        graph.addVertex(vertex)

    for beginning, end, cost, distance in newEdgesH:
        graph.addEdge(beginning, end, cost, distance)

    # 3. Add edges between consecutive edges on boundaries
    return graph


obstacles = [
    [[1, 0], [2, -1], [2, 1]],
    [[3, 0], [2.1, -1], [2.1, 1]],
]
costs = [3, 3]

tree = makeHorizontalPersistentTree(obstacles, costs, horizontalRelation, equality)
tree.printTree(3)
print(tree.accessRange([[2.7, -0.5], [2.7, -0.5]], [[2.7, 0.5], [2.7, 0.5]], 2.5))
