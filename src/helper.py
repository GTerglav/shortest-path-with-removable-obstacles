import math
import sys
from matplotlib import pyplot as plt
import numpy as np

# smallest positive number
epsilon = sys.float_info.epsilon


# euclidean distance
def distance(p1, p2):
    return math.sqrt(abs(p1[0] - p2[0]) ** 2 + abs(p1[1] - p2[1]) ** 2)


# returns angle and distance between v and p. For sorting with sweep algo
def compareAngle(v, p):
    angle = math.atan2(p[1] - v[1], p[0] - v[0])
    dist = distance(v, p)
    return (-angle, dist)


# Checks if halfline v p intersects obstacle segment start end
def intersectLine(v, p, start, end):
    # Calculate the direction vectors of the edge (start, end) and the line (v, p)
    b0 = end[0] - start[0]
    b1 = end[1] - start[1]
    a0 = p[0] - v[0]
    a1 = p[1] - v[1]

    determinant = b0 * a1 - b1 * a0

    # Check if the determinant is close to zero, indicating collinearity
    if abs(determinant) < 1e-10:
        if ccw(v, p, start) == 0:
            return min(distance(v, start), distance(v, end))
        else:
            return math.inf
    else:
        # Calculate the parameters t and u for the intersection point
        u = ((v[0] - start[0]) * a1 - (v[1] - start[1]) * a0) / determinant
        t = (b1 * (v[0] - start[0]) + b0 * (start[1] - v[1])) / determinant

        # Check if the intersection point lies within the line segment vp
        if t >= 0 and 0 < u < 1:
            return t
        else:
            return math.inf


# Check if line segment (p1, q1) intersects line segment (p2, q2)
def intersectSegments(p1, q1, p2, q2):
    # orientation of points p,q,r
    def orientation(p, q, r):
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if val == 0:
            return 0  # Collinear
        return 1 if val > 0 else -1  # Clockwise or Counterclockwise

    # Check if point q lies on segment p-r
    def onSegment(p, q, r):
        if (
            q[0] <= max(p[0], r[0])
            and q[0] >= min(p[0], r[0])
            and q[1] <= max(p[1], r[1])
            and q[1] >= min(p[1], r[1])
        ):
            return True
        return False

    # Find orientations of points
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # General case: check for proper intersection
    if o1 != o2 and o3 != o4:
        # Check if the segments share an endpoint but do not overlap along the line
        if (
            (o1 == 0 and not onSegment(p1, p2, q1))
            or (o2 == 0 and not onSegment(p1, q2, q1))
            or (o3 == 0 and not onSegment(p2, p1, q2))
            or (o4 == 0 and not onSegment(p2, q1, q2))
        ):
            return False
        return True

    return False


# Check if point q lies on line segment pr
def onSegment(p, q, r):
    return min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and min(p[1], r[1]) <= q[
        1
    ] <= max(p[1], r[1])


# Check if the line segment between p and q intersects the obstacle
# Assuming obstacle is a list of vertices representing the obstacle polygon
def intersect(p, q, obstacle):
    for i in range(len(obstacle)):
        start = obstacle[i]
        end = obstacle[
            (i + 1) % len(obstacle)
        ]  # Next vertex in the obstacle (considering it's a closed polygon)
        if intersect2(p, q, start, end):
            return True
    return False


# Check if segments p1,p2 and p3,p4 intersect
def intersect2(p1, p2, p3, p4):
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


# orientation of points
def ccw(p1, p2, p3):
    if p3 is None:
        return 1
    else:
        return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])


# are v,w in the same obstacle
def inSameObstacleSegment(v, w, obstacles, costs):
    for obstacle, cost in zip(obstacles, costs):
        if any(np.array_equal(point, v) for point in obstacle) and any(
            np.array_equal(point, w) for point in obstacle
        ):
            return obstacle, cost
    return None


def areNeighboursInObstacle(v, w, obstacle):
    index_v = np.where(np.all(obstacle == v, axis=1))[0][0]
    index_w = np.where(np.all(obstacle == w, axis=1))[0][0]
    n = len(obstacle)

    return (
        abs(index_v - index_w) == 1
        or (index_v == 0 and index_w == n - 1)
        or (index_w == 0 and index_v == n - 1)
    )


# Root is tree of obstacle edges
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
            if intersect2(v, w, start, end):
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


def printGraph(graph):
    for vertex, neighbors in graph.vertices.items():
        print(f"Vertex {vertex}: Neighbors {neighbors}")


def plotViabilityGraph(start, goal, obstacles, graph, shortestPath=None):
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
            plt.plot([p1[0], p2[0]], [p1[1], p2[1]], "b-", linewidth=0.2)

    # Plot start and goal
    plt.plot(*zip(*np.vstack((start, goal))), "ro")

    # Plot shortest path if provided
    if shortestPath:
        path_array = np.array(shortestPath)
        plt.plot(path_array[:, 0], path_array[:, 1], "r-")

    # Set aspect ratio and display
    plt.gca().set_aspect("equal", adjustable="box")
    plt.show()


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
        # path = [(x, y) for x, y, _ in shortestPath]
        pathArray = np.array(shortestPath)
        plt.plot(pathArray[:, 0], pathArray[:, 1], "r-")

    # Set aspect ratio and display
    # plt.gca().set_aspect("equal", adjustable="box")
    plt.legend()
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Graph with Obstacles")
    plt.grid(True)
    plt.show()


# For the sweep algorithm, since its slightly different
def plotPointsAndObstaclesSweep(
    start, goal, obstacles, budget, costs, epsilon, shortestPath=None
):
    plt.figure()

    # Plot obstacles
    for i, obs in enumerate(obstacles):
        xCoords = [point[0] for point in obs]
        yCoords = [point[1] for point in obs]
        plt.fill(*zip(*obs), color="gray", alpha=0.5)
        for vertex in obs:
            plt.plot(*vertex, "ko")  # Plot obstacle vertices

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
    plt.plot(*start, "ro", label="Start")
    plt.plot(*goal, "bo", label="Goal")

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

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.show()


def plotProblem(start, goal, obstacles, budget, costs):
    plt.figure()

    # Plot obstacles
    for i, obs in enumerate(obstacles):
        xCoords = [point[0] for point in obs]
        yCoords = [point[1] for point in obs]
        plt.fill(*zip(*obs), color="gray", alpha=0.5)
        for vertex in obs:
            plt.plot(*vertex, "ko")  # Plot obstacle vertices

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
    plt.plot(*start, "ro", label="Start")
    plt.plot(*goal, "bo", label="Goal")

    plt.text(
        0.5,
        1.05,
        f"Budget: {budget}",
        transform=plt.gca().transAxes,
        ha="center",
        fontsize=14,
    )

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.show()


# Returns the two neighbors of point in obstacle
def findObstacleEdges(obstacles, point, obstacleIndex):
    obstacle = obstacles[obstacleIndex]
    if point in obstacle:
        index = obstacle.index(point)
        prev_index = (index - 1) % len(obstacle)
        next_index = (index + 1) % len(obstacle)
        return [obstacle[prev_index], obstacle[next_index]]
    return None


# does an edge [v,u] have positive slope
def edgeOrientiation(edge):
    [[v1, v2], [u1, u2]] = edge
    if v1 < u1:
        return v2 < u2
    else:
        return u2 < v2


# Finds intersecting point of segments A1,A2 and B1,B2
def intersectionPoint(A1, A2, B1, B2):
    # Extract coordinates
    x1, y1 = A1
    x2, y2 = A2
    x3, y3 = B1
    x4, y4 = B2

    # Compute determinants
    denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

    if denom == 0:
        # Lines are parallel or coincident
        return None

    # Compute intersection point
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom

    # Check if the intersection point is on both line segments
    if (
        min(x1, x2) <= px <= max(x1, x2)
        and min(y1, y2) <= py <= max(y1, y2)
        and min(x3, x4) <= px <= max(x3, x4)
        and min(y3, y4) <= py <= max(y3, y4)
    ):
        return [px, py]

    return None


# Returns list of obstacle vertices
def obstacleVertices(obstacles):
    vertices = []
    for obstacle in obstacles:
        for vertex in obstacle:
            vertices.append(vertex)
    return vertices


# Input are obstacle edges that intersect a line segment
# Returns cost of line segment
def costHelper(edges):
    totalCost = 0
    costDict = {}
    for edge in edges:
        node, edge = edge[0], edge[1]
        cost = node.val[2]
        obstacleIndex = node.val[3]
        if obstacleIndex in costDict:
            costDict.pop(obstacleIndex)
        else:
            costDict[obstacleIndex] = cost
        totalCost += cost

    if len(costDict) == 0:
        totalCost /= 2
    else:
        singleCosts = 0
        for obstacleName in costDict.keys():
            singleCosts += costDict[obstacleName]
        totalCost = (totalCost - singleCosts) / 2 + singleCosts

    return totalCost


# functions for rotating the points in plane
def rotatepoint(x, y, alpha):
    # Convert angle alpha to radians
    alphaRad = math.radians(alpha)

    # Rotation matrix components
    cosAlpha = math.cos(alphaRad)
    sinAlpha = math.sin(alphaRad)

    # New coordinates
    xNew = x * cosAlpha + y * sinAlpha
    yNew = -x * sinAlpha + y * cosAlpha

    return [xNew, yNew]


# print(rotatepoint(2.1407, -0.27919, 288))
# print(rotatepoint(1.949733922046848, 0.30858725745354826, 288))


# The other way
def revertpoint(xNew, yNew, alpha):
    alphaRad = math.radians(alpha)
    cosAlpha = math.cos(alphaRad)
    sinAlpha = math.sin(alphaRad)
    x = xNew * cosAlpha - yNew * sinAlpha
    y = xNew * sinAlpha + yNew * cosAlpha

    return [x, y]


def rotationalEquality(p1, p2, angle):
    x1, y1 = p1
    x2, y2 = p2
    newX1, newY1 = rotatepoint(x1, y1, angle)
    newX2, newY2 = revertpoint(newX1, newY1, angle)
    diff = abs(x2 - newX2) + abs(y2 - newY2)
    return diff < 0.000000001


# Second highest element of list
def secondHighest(arr):
    if len(arr) < 2:
        return None
    first, second = float("-inf"), float("-inf")
    for number in arr:
        if number >= first:
            first, second = number, first
        elif first >= number >= second:
            second = number
    return second if second != float("-inf") else None


# Returns True if vertices v,u are very close
def verticesDifferent(v, u):
    return abs(v[0] - u[0]) + abs(v[1] - u[1]) > 0.000000001
