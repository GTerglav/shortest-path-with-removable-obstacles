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

import sys
from functools import cmp_to_key

# Viability graph algoritm as before
from sweepViabilityGraph import Graph, viableVerticesFromV, viable, viabilityGraph


######################## Now to sparsify the graph ###############

# Very small positive number
epsilon = sys.float_info.epsilon

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


# a = [[1, 0], [2, -1]]
# b = [[1, 0], [2, 1]]

# print(horizontalRelation(a, b))


# Make tree of obstacle segments for vertical split lines. "Time" is on y-axis
def makeVerticalPersistentTree(obstacles, costs, relation, equality):
    tree = persistentRBTree(relation, equality)

    # Just a list of all points with costs
    points = []
    for i, obstacle in enumerate(obstacles):
        for vertex in obstacle:
            points.append([vertex, costs[i], i])

    # Sort them by y, first has lowest
    sortedPoints = sorted(points, key=lambda point: point[0][1])

    queue = []
    for point in sortedPoints:

        while queue != [] and queue[0][2] < point[0][1]:
            tree.insert(queue[0][0], queue[0][1], queue[0][2])
            queue.pop(0)

        # two neighbors in obstacle
        neighbors = helper.findObstacleEdges(obstacles, point[0])
        if neighbors:
            for nh in neighbors:
                # not considering "= case" since the obstacle segments in such cases are not positive or negative
                # insert
                if nh[1] > point[0][1]:
                    key = [point[0], nh]  # keys are endpoints of segment
                    # if nh[0] != point[0][0]: # Actually I dont want to ignore vertical for cost computing
                    # val is 1st vertex of edge, 2nd, cost of obstacle of edge, index of obstacle
                    # add insertions to queue, because they happend at time+epsilon
                    queue.append(
                        [key, [point[0], nh, point[1], point[2]], point[0][1] + epsilon]
                    )
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
            points.append([vertex, costs[i], i])

    # Sort them by x, first has lowest
    sortedPoints = sorted(points, key=lambda point: point[0][0])

    queue = []  # Queue for delayed insertion
    for point in sortedPoints:
        while queue != [] and queue[0][2] < point[0][1]:
            tree.insert(queue[0][0], queue[0][1], queue[0][2])
            queue.pop(0)

        neighbors = helper.findObstacleEdges(obstacles, point[0])
        if neighbors:
            for nh in neighbors:
                if nh[0] > point[0][0]:
                    key = [point[0], nh]
                    queue.append(
                        [key, [point[0], nh, point[1], point[2]], point[0][1] + epsilon]
                    )
                elif nh[0] < point[0][0]:
                    key = [nh, point[0]]
                    tree.delete(key, point[0][0])
    return tree


# TODO
# Computes cost of edge (v,u)
def costFunction(v, u, tree):
    # get all edges that intersect (u,v)
    if v[0] == u[0]:  # (u,v) vertical
        lb = [v, [v[0], v[1] + epsilon]]
        hb = [u, [u[0], u[1] + epsilon]]
        if v[1] < u[1]:
            edges = tree.accessRange(lb, hb, v[0])
        else:
            edges = tree.accessRange(hb, lb, v[0])
    elif v[1] == u[1]:  # (u,v) horiznotal
        lb = [v, [v[0], v[1] + epsilon]]
        hb = [u, [u[0], u[1] + epsilon]]
        if v[0] < u[0]:
            edges = tree.accessRange(lb, hb, v[1])

        else:
            edges = tree.accessRange(hb, lb, v[1])

    return helper.costHelper(edges)


# Finds first positive or negative slope segmenet that intersects (v,u).
# (v,u) is horizontal or vertical
def obstacleSegment(v, u, tree, verticalRelation, horizontalRelation, positive=True):
    # Need for sorting
    def verticalEdgeComparison(edge1, edge2):
        if verticalRelation(edge1, edge2):
            return -1
        elif verticalRelation(edge2, edge1):
            return 1
        else:
            return 0

    def horizontalEdgeComparison(edge1, edge2):
        if horizontalRelation(edge1, edge2):
            return -1
        elif horizontalRelation(edge2, edge1):
            return 1
        else:
            return 0

    verticalFunction = cmp_to_key(verticalEdgeComparison)
    horizontalFunction = cmp_to_key(horizontalEdgeComparison)

    # get all edges that intersect (u,v)
    if v[0] == u[0]:  # (u,v) vertical
        lb = [v, [v[0], v[1] + epsilon]]
        hb = [u, [u[0], u[1] + epsilon]]
        if v[1] < u[1]:
            edges = tree.accessRange(lb, hb, v[0])
            edges.sort(key=lambda x: horizontalFunction(x[1]), reverse=True)
        else:
            edges = tree.accessRange(hb, lb, v[0])
            edges.sort(key=lambda x: horizontalFunction(x[1]))
    elif v[1] == u[1]:  # (u,v) horiznotal
        lb = [v, [v[0], v[1] + epsilon]]
        hb = [u, [u[0], u[1] + epsilon]]
        if v[0] < u[0]:
            edges = tree.accessRange(lb, hb, v[1])
            edges.sort(key=lambda x: verticalFunction(x[1]), reverse=True)
        else:
            edges = tree.accessRange(hb, lb, v[1])
            edges.sort(key=lambda x: verticalFunction(x[1]))

    # v is projection of u so we need the edge closest to u with pos/neg slope.
    for edge in edges:
        node, edge = edge[0], edge[1]
        cost = node.val[2]
        obstacleIndex = node.val[3]
        if positive and helper.edgeOrientiation(edge):
            return [edge, cost, obstacleIndex]
        if (not positive) and (not helper.edgeOrientiation(edge)):
            return [edge, cost, obstacleIndex]
    return None


# TODO AVOID ADDING DUPLICATE VERTICES AND EDGES!!!!!!!!!!
# This new graph should start as the viability graph
# vertical == True then vertical split line otherwise horizontal
# We need both trees. if vertical we need vertical tree for all costs,
# except when connecting steiner vertices at the end.
def Recurse(vertices, obstacles, costs, budget, tree1, tree2, vertical=True):
    # This function is going to returna list if all edges and vertices that need to be added to our viability graph
    steinerVertices = []  # new vertices that are on split line
    newVertices = []  # new vertices not on split line
    newEdges = []

    if len(vertices) <= 1:
        return [[], []]

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
            projectedVertex = [median, vertex[1]]
            splitLine = [[median, math.inf], [median, -math.inf]]
        else:
            projectedVertex = [vertex[0], median]
            splitLine = [[math.inf, median], [-math.inf, median]]
        cost = costFunction(vertex, projectedVertex, tree1)

        # a) First add the projected vertex to graph
        if cost <= budget:
            # newGraph.addVertex(projectedVertex)
            # newGraph.addEdge(vertex,projectedVertex, cost, helper.distance(vertex,projectedVertex))
            steinerVertices.append(projectedVertex)
            newEdges.append(
                (
                    vertex,
                    projectedVertex,
                    cost,
                    helper.distance(vertex, projectedVertex),
                )
            )

        # b),c) Check first intersecting obstacle segments
        posSegment = obstacleSegment(
            projectedVertex, vertex, tree1, verticalRelation, horizontalRelation, True
        )
        negSegment = obstacleSegment(
            projectedVertex, vertex, tree1, verticalRelation, horizontalRelation, False
        )

        # If segment intersects splitline, add bypass vertices
        if posSegment is not None:
            bypass1 = helper.intersectionPoint(
                posSegment[0][0], posSegment[0][1], vertex, projectedVertex
            )
            bypass2 = helper.intersectionPoint(
                posSegment[0][0], posSegment[0][1], splitLine[0], splitLine[1]
            )
            if bypass2:
                newVertices.append(bypass1)
                steinerVertices.append(bypass2)
                newEdges.append(
                    (
                        bypass1,
                        bypass2,
                        0,
                        helper.distance(bypass1, bypass2),
                    )
                )
                newEdges.append(
                    (
                        vertex,
                        bypass1,
                        costFunction(vertex, bypass1, tree1),
                        helper.distance(vertex, bypass1),
                    )
                )

        if negSegment is not None:
            bypass1 = helper.intersectionPoint(
                negSegment[0][0], negSegment[0][1], vertex, projectedVertex
            )
            bypass2 = helper.intersectionPoint(
                negSegment[0][0], negSegment[0][1], splitLine[0], splitLine[1]
            )
            if bypass2:
                newVertices.append(bypass1)
                steinerVertices.append(bypass2)
                newEdges.append(
                    (
                        bypass1,
                        bypass2,
                        0,
                        helper.distance(bypass1, bypass2),
                    )
                )
                newEdges.append(
                    (
                        vertex,
                        bypass1,
                        costFunction(vertex, bypass1, tree1),
                        helper.distance(vertex, bypass1),
                    )
                )

        # d) Connect adjacent new vertices on the split line
        if vertical:
            sortingParameter = 1
        else:
            sortingParameter = 0
        sortedSteinerVertices = sorted(
            steinerVertices, key=lambda vertex: vertex[sortingParameter]
        )
        for i in range(len(sortedSteinerVertices) - 1):
            v = sortedSteinerVertices[i]
            u = sortedSteinerVertices[i + 1]
            cost = costFunction(v, u, tree2)
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
            if vertex[0] < median:
                leftVertices.append(vertex)
            elif vertex[0] > median:
                rightVertices.append(vertex)
        else:
            if vertex[1] < median:
                leftVertices.append(vertex)
            elif vertex[1] > median:
                rightVertices.append(vertex)

    return [
        newVertices
        + steinerVertices
        + Recurse(rightVertices, obstacles, costs, budget, tree1, tree2, vertical)[0]
        + Recurse(leftVertices, obstacles, costs, budget, tree1, tree2, vertical)[0],
        newEdges
        + Recurse(rightVertices, obstacles, costs, budget, tree1, tree2, vertical)[1]
        + Recurse(leftVertices, obstacles, costs, budget, tree1, tree2, vertical)[1],
    ]


# TODO
def sparsify(start, goal, obstacles, costs, budget):
    # (-1). Initialize graph and all points
    graph = Graph()
    vertices = [start] + [goal] + helper.obstacleVertices(obstacles)

    # 0. Make trees of obstacle segments

    horizontalTree = makeHorizontalPersistentTree(
        obstacles, costs, horizontalRelation, equality
    )
    verticalTree = makeVerticalPersistentTree(
        obstacles, costs, verticalRelation, equality
    )

    # 1. Step add new edges and vertices, vertical
    newVerticesV, newEdgesV = Recurse(
        vertices, obstacles, costs, budget, verticalTree, horizontalTree, True
    )

    for vertex in newVerticesV:
        graph.addVertex(vertex)

    for beginning, end, cost, distance in newEdgesV:
        graph.addEdge(beginning, end, cost, distance)

    # 2. Step add new edges and vertices, horizontal
    newVerticesH, newEdgesH = Recurse(
        vertices,
        obstacles,
        costs,
        budget,
        horizontalTree,
        verticalTree,
        False,
    )

    for vertex in newVerticesH:
        graph.addVertex(vertex)

    for beginning, end, cost, distance in newEdgesH:
        graph.addEdge(beginning, end, cost, distance)

    # 3. Add edges between consecutive edges on boundaries

    for obstacle in obstacles:
        for i in range(len(obstacle)):
            u = obstacle[i]
            v = obstacle[
                (i + 1) % len(obstacle)
            ]  # To connect the last point to the first point
            graph.addEdge(u, v, 0, helper.distance(u, v))

    return graph


start = [0, 0]
goal = [4, 0]
obstacles = [
    [[1, 0], [2, -1], [2, 1]],
    [[3, 0], [2.1, -1], [2.1, 1]],
]
costs = [3, 3]
budget = 3

# obstacles = [
#     [[1, 0], [2, -1], [3, 0], [2, 1]],
# ]
# costs = [3]

graph = sparsify(start, goal, obstacles, costs, budget)
helper.printGraph(graph)

t = {1, 2}
h = {2, 3}
print(t.union(h))
# tree = makeVerticalPersistentTree(obstacles, costs, verticalRelation, equality)
# tree.printTree(0.1)
# print(
#     tree.accessRange(
#         [[0.5, -1], [0.5, -1 + epsilon]], [[2.5, -1], [2.5, -1 + epsilon]], -1
#     )
# )
# print("first neg slope is:")
# print(
#     obstacleSegment(
#         [2.5, -0.9], [0.5, -0.9], tree, verticalRelation, horizontalRelation, False
#     )
# )

# print("cost is:")
# print(costFunction([0, 0], [4, 0], tree))
