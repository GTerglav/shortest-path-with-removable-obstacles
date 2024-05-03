import math

import numpy as np
from AVLTree import AVLTree, TreeNode

class Graph:
    def __init__(self):
        self.vertices = {}
        
    def addVertex(self, index):
        self.vertices[index] = []
        
    def addEdge(self, start, end, cost, length):
        self.vertices.setdefault(start, []).append((end, cost, length))
        self.vertices.setdefault(end, []).append((start, cost, length))

def distance(p1,p2):
    return math.sqrt(abs(p1[0]-p2[0])**2 + abs(p1[1]-p2[1])**2)

def compareAngle(v, p):
    angle = math.atan2(p[1] - v[1], p[0] - v[0])
    dist = distance(v, p)
    return (angle, dist)

#Function that checks if halfline v p intersects obstacle segment start end
def intersectLine(v, p, start, end):
    # Calculate the direction vectors of the edge (start, end) and the line (v, p)
    dx_edge = end[0] - start[0]
    dy_edge = end[1] - start[1]
    dx_line = p[0] - v[0]
    dy_line = p[1] - v[1]

    # Calculate the determinant of the 2x2 matrix formed by the direction vectors
    determinant = dx_edge * dy_line - dy_edge * dx_line
    
    # Check if the determinant is close to zero, indicating collinearity
    if abs(determinant) < 1e-10:
        # Check if the points are collinear
        if abs(dx_edge * (v[1] - start[1]) - dy_edge * (v[0] - start[0])) < 1e-10:
            # Check if the intersection point lies within the line segment vp
            t = ((v[0] - start[0]) * dy_line - (v[1] - start[1]) * dx_line) / determinant
            u = -((v[0] - start[0]) * dy_edge - (v[1] - start[1]) * dx_edge) / determinant
            if t >= 0 and 0 <= u <= 1:
                return True
    else:
        # Calculate the parameters t and u for the intersection point
        t = ((v[0] - start[0]) * dy_line - (v[1] - start[1]) * dx_line) / determinant
        u = -((v[0] - start[0]) * dy_edge - (v[1] - start[1]) * dx_edge) / determinant

        # Check if the intersection point lies within the line segment vp
        if t >= 0 and 0 <= u <= 1:
            return True

    return False

def intersectSegments(p1, q1, p2, q2):
    # Check if line segment (p1, q1) intersects line segment (p2, q2)
    # You can use various algorithms to check for intersection, such as cross product method
    # Here, I'll use a simple approach based on orientation of points
    # This assumes that p1, q1, p2, and q2 are tuples representing points (x, y)
    def orientation(p, q, r):
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if val == 0:
            return 0  # Collinear
        return 1 if val > 0 else -1  # Clockwise or Counterclockwise

    # Find orientations of points
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # General case: check for proper intersection
    if o1 != o2 and o3 != o4:
        return True

    # Special cases: segments are collinear and overlapping
    if o1 == 0 and onSegment(p1, p2, q1):
        return True
    if o2 == 0 and onSegment(p1, q2, q1):
        return True
    if o3 == 0 and onSegment(p2, p1, q2):
        return True
    if o4 == 0 and onSegment(p2, q1, q2):
        return True

    return False

def onSegment(p, q, r):
    # Check if point q lies on line segment pr
    return min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and min(p[1], r[1]) <= q[1] <= max(p[1], r[1])

def intersect(p, q, obstacle):
    # Check if the line segment between p and q intersects the obstacle
    # Assuming obstacle is a list of vertices representing the obstacle polygon
    # Iterate over each edge of the obstacle
    for i in range(len(obstacle)):
        start = obstacle[i]
        end = obstacle[(i + 1) % len(obstacle)]  # Next vertex in the obstacle (considering it's a closed polygon)
        if intersectSegments(p, q, start, end):
            return True
    return False

def viable(p, q, obstacles, costs, budget):
    # Check if the line segment between p and q intersects any obstacle
    totalCost = 0
    for obstacle, cost in zip(obstacles, costs):
        if intersect(p, q, obstacle):
            totalCost += cost  # Reduce the budget by the cost of the obstacle
    return totalCost

def viableVerticesFromV(v, points, obstacles, costs, budget):
    #here sort vertices accoring to angle from v and distance from v
    sortedVertices = sorted(points, key=lambda x: compareAngle(v, x))
    
    T = AVLTree()
    root = None

    #here need to store obstacle edges that intersect with halfline form v
    for obstacle in obstacles:
        for i in range(len(obstacle)):
            start = obstacle[i]
            end = obstacle[(i + 1) % len(obstacle)]
            if intersectLine(v, (v[0]+1,v[1]), start, end):
                T.insert(root,(start,end))

    W = set()
    for w in sortedVertices:
        if viable(v, w, points, costs, budget) <= budget:
            W.add(w)

    
    return None

def viabilityGraph(start, goal, obstacles, costs, budget):
    points = np.vstack((start, goal, np.vstack(obstacles)))
    graph = Graph()
    for i, p1 in enumerate(points):
        print(i,p1)
        graph.addVertex(i)
        viable = viableVerticesFromV(p1, points, obstacles, costs, budget)
        for j,p2,totalCost in viable:
            graph.addEdge(i,j,totalCost,distance(p1,p2))

    return graph



