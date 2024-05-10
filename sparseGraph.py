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


def sparsify(graph, start, goal, obstacles, costs, budget):
    xCoordinates = []
    for vertex in graph.vertices:
        xCoordinates.append(vertex[0])
        
    xMedian = statistics.median(xCoordinates)

    return "TODO"
