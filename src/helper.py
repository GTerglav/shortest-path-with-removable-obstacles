import math

from matplotlib import pyplot as plt
import numpy as np


def distance(p1, p2):
    return math.sqrt(abs(p1[0] - p2[0]) ** 2 + abs(p1[1] - p2[1]) ** 2)


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
        # t = ((v[0] - start[0]) * b1 - (v[1] - start[1]) * b0) / determinant
        u = ((v[0] - start[0]) * a1 - (v[1] - start[1]) * a0) / determinant
        t = (b1 * (v[0] - start[0]) + b0 * (start[1] - v[1])) / determinant

        # Check if the intersection point lies within the line segment vp
        if t >= 0 and 0 < u < 1:
            return t
        else:
            return math.inf


def intersectSegments(p1, q1, p2, q2):
    # Check if line segment (p1, q1) intersects line segment (p2, q2)
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


def onSegment(p, q, r):
    # Check if point q lies on line segment pr
    return min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and min(p[1], r[1]) <= q[
        1
    ] <= max(p[1], r[1])


def intersect(p, q, obstacle):
    # Check if the line segment between p and q intersects the obstacle
    # Assuming obstacle is a list of vertices representing the obstacle polygon
    # Iterate over each edge of the obstacle
    for i in range(len(obstacle)):
        start = obstacle[i]
        end = obstacle[
            (i + 1) % len(obstacle)
        ]  # Next vertex in the obstacle (considering it's a closed polygon)
        if intersect2(p, q, start, end):
            return True
    return False


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


def ccw(p1, p2, p3):
    if p3 is None:
        return 1
    else:
        return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])


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


def costOfSegment(v, w, root):
    if root is None:
        return 0

    total_cost = 0

    # Traverse the AVL tree to find edges intersecting with the segment
    stack = [root]
    while stack:
        node = stack.pop()
        if node is not None:
            start, end, _, cost = node.key
            # Check if the edge intersects with the segment vw
            if intersect2(v, w, start, end):
                total_cost += cost
            # Traverse left and right subtrees
            stack.append(node.left)
            stack.append(node.right)

    return total_cost


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


def plotPointsAndObstaclesSweep(start, goal, obstacles, shortestPath=None):
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
        path = [(x, y) for x, y, _ in shortestPath]
        pathArray = np.array(path)
        plt.plot(pathArray[:, 0], pathArray[:, 1], "r-")

    # Set aspect ratio and display
    # plt.gca().set_aspect("equal", adjustable="box")
    plt.legend()
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Graph with Obstacles")
    plt.grid(True)
    plt.show()
