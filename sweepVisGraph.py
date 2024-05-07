import math

from matplotlib import pyplot as plt
import numpy as np
from AVLTree import AVLTree, TreeNode

from dijkstra import dijkstra
import helper
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
        self.vertices.setdefault(end_tuple, []).append((start_tuple, cost, length))


############################### Viability algorithm #############################################


def viable(p, w, wMinusOne, costToWMinusOne, root, obstacles, costs):
    # If w and w_-1 are not colinear then just sum up all the obstacles on the path
    if (wMinusOne is None) or (helper.ccw(p, w, wMinusOne)) != 0:
        if root is None:
            return 0

        totalCost = 0
        count = 0

        # Traverse the AVL tree to find edges
        stack = [root]
        while stack:
            node = stack.pop()
            if node is not None:
                start, end, _, cost = node.key
                if np.array_equal(start, w) and np.array_equal(end, w):
                    totalCost += cost
                    count += 1
                stack.append(node.left)
                stack.append(node.right)

        # Divide the total cost by 2 if there are an even number of edges
        if count % 2 == 0:
            totalCost /= 2
        else:
            totalCost = (totalCost - cost) / 2 + cost
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

    # Notes to self:
    # here need to store obstacle edges that intersect with halfline form v
    # But I dont think i need to store them in avl tree because i have to check them all for intersection anyway
    # not just the leftmost one like in visibility algorithm. Ok maybe for removal and insertion AVL is nice.
    # But i got through all obstacles for every halfline so this does nothing its just naive right?
    # No i just go through incident edges for every halfline. Just at start i got through all obstacles.
    # So this should be faster

    # Insert edges that intersect into tree
    for j, obstacle in enumerate(obstacles):
        for i in range(len(obstacle)):
            start = obstacle[i]
            end = obstacle[(i + 1) % len(obstacle)]
            t = helper.intersectLine(v, (v[0] + 1, v[1]), start, end)
            if t < math.inf:
                # So our tree stores the following info about edges: start, end, distance from v and cost of whole obstacle.
                root = T.insert(root, (start, end, t, costs[j]))

    W = set()
    wMinusOne = None
    costToWMinusOne = 0
    for w in sortedVertices:
        # tle morš vedt ceno robov in še greš samo čez robove v V.
        costOfPathToW = viable(v, w, wMinusOne, costToWMinusOne, root, obstacles, costs)
        print(v, w, costOfPathToW)
        if costOfPathToW <= budget:
            W.add((tuple(w), costOfPathToW))

        # need to delete from T edge that line on counter clockwise side of line
        # But all edges that include w should be on the counter clockwise side so i can just delete them all
        root = T.deleteByVertex(root, w)

        # need to insert into T edges that include w and lie on the clockwise side of the line
        for k, obstacle in enumerate(obstacles):
            for i in range(len(obstacle)):
                start = obstacle[i]
                end = obstacle[(i + 1) % len(obstacle)]
                if (helper.ccw(v, w, start) <= 0 and np.array_equal(end, w)) or (
                    helper.ccw(v, w, end) <= 0 and np.array_equal(start, w)
                ):
                    root = T.insert(root, (start, end, helper.distance(v, w), costs[k]))
        # we need to store w_-1
        wMinusOne = w
        costToWMinusOne = costOfPathToW
    return W


###TODO###
def viabilityGraph(start, goal, obstacles, costs, budget):
    points = np.vstack((start, goal, np.vstack(obstacles)))
    graph = Graph()
    for p1 in points:
        graph.addVertex(p1)
        viable = viableVerticesFromV(p1, points, obstacles, costs, budget)
        for p2, totalCost in viable:
            graph.addEdge(p1, p2, totalCost, helper.distance(p1, p2))

    return graph


def plotGraph(graph, start, goal, obstacles):
    # Plot edges
    for vertex, edges in graph.vertices.items():
        for edge in edges:
            start_point = np.array(vertex)
            end_point, _, _ = edge
            end_point = np.array(end_point)
            plt.plot(
                [start_point[0], end_point[0]],
                [start_point[1], end_point[1]],
                "b-",
                linewidth=0.6,
            )

    # Plot obstacles
    for obstacle in obstacles:
        x_coords = [point[0] for point in obstacle]
        y_coords = [point[1] for point in obstacle]
        plt.fill(x_coords, y_coords, "gray")

    # Plot start and end vertices
    plt.scatter(start[0], start[1], color="green", label="Start")
    plt.scatter(goal[0], goal[1], color="red", label="Goal")

    plt.legend()
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Graph with Obstacles")
    plt.grid(True)
    plt.show()


def main(problem):
    start = problem.start
    goal = problem.goal
    obstacles = problem.obstacles
    costs = problem.costs
    budget = problem.budget
    epsilon = problem.epsilon

    graph = viabilityGraph(start, goal, obstacles, costs, budget)
    # copiedGraph = createCopiesOfGraph(graph, budget, epsilon)

    # start_vertex = -1
    # targetVertex = -2
    # shortestPath = dijkstra(copiedGraph, start_vertex, targetVertex)

    # if shortestPath:
    #     nicePath = convertPathToCoordinates(
    #         shortestPath, obstacles, start, goal, budget, epsilon
    #     )
    #     print("Shortest path:", nicePath)

    # plotPointsAndObstacles(start, goal, obstacles, nicePath)
    plotGraph(graph, start, goal, obstacles)


if __name__ == "__main__":
    main(problems.problem2)
