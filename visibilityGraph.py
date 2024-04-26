import math
import numpy as np
import matplotlib.pyplot as plt


class Graph:
    def __init__(self):
        self.vertices = {}
        
    def addVertex(self, index):
        self.vertices[index] = []
        
    def addEdge(self, start, end, cost):
        self.vertices.setdefault(start, []).append((end, cost))

"""checks if lines p1p2 and p3p4 intersect"""
def intersect(p1, p2, p3, p4):
    """ Orientation of points abc"""
    def ccw(a, b, c):
        return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])

    return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)

def visibilityGraph(start, goal, obstacles, costs, budget):
    """ Construct visibility graph """
    points = np.vstack((start, goal, np.vstack(obstacles)))
    print(points)
    graph = Graph()
    for i, p1 in enumerate(points):
        print(i,p1)
        graph.addVertex(i)
        for j, p2 in enumerate(points):
            if i != j:
                # This is surely not efficient
                # Check if p1 and p2 belong to the same obstacle segment
                same_segment = False
                for obs in obstacles:
                    if ((obs == p1).all() or (obs == p2).all()) and np.any(np.all(obs == p1, axis=1) & np.all(obs == p2, axis=1)) and (np.abs(np.where(np.all(obs == p1, axis=1))[0] - np.where(np.all(obs == p2, axis=1))[0]) == 1):
                        same_segment = True
                        break
                # If not in the same segment, check for intersection
                if not same_segment:
                    totalCost = 0
                    for l, obs in enumerate(obstacles):
                        for k in range(len(obs)-1):
                            if intersect(p1, p2, obs[k], obs[k+1]):
                                totalCost += costs[l]
                                break
                    if totalCost <= budget:
                        graph.addEdge(i,j,totalCost)
    return graph

def createCopiesOfGraph(graph, budget, epsilon):
    newGraph = Graph()
    for i, neighbors in graph.vertices.items():
        numCopies = math.ceil(budget / epsilon)  
        for k in range(numCopies):
            new_i = (i * numCopies) + k  
            for j, cost in neighbors:
                new_k = math.ceil(k + cost) #index of j with which we want to connect
                new_j = ((j)*numCopies+new_k) #so thid vertex is indexed as j_k' and we connect it to i_k
                newGraph.addEdge(new_i, new_j, 0)  #now costs are 0 we dont need them
    newGraph.addVertex("start")
    newGraph.addVertex("sink")

    #We want to create a start and a sink and connect them to all of their copies.
    for i in range(numCopies):
        newGraph.addEdge("start",i, 0)
        newGraph.addEdge("sink",(len(graph.vertices)-1)*numCopies-i, 0)
    return newGraph

def plotVisibilityGraph(start, goal, obstacles, graph):
    """ Plot the visibility graph """
    plt.figure()
    
    # Plot obstacles
    for obs in obstacles:
        plt.fill(*zip(*obs), color='gray', alpha=0.5)
        for vertex in obs:
            plt.plot(*vertex, 'ko')  # Plot obstacle vertices
    
    # Plot edges
    for start_vertex, edges in graph.vertices.items():
        p1 = np.vstack((start, goal, np.vstack(obstacles)))[start_vertex]
        for end_vertex, _ in edges:
            p2 = np.vstack((start, goal, np.vstack(obstacles)))[end_vertex]
            plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'b-')
    
    # Plot start and goal
    plt.plot(*zip(*np.vstack((start, goal))), 'ro')
    
    # Set aspect ratio and display
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()


# Define points and obstacles
start = np.array([1, 2.5])
goal = np.array([9, 2])
obstacles = [[[3, 3], [4, 3], [4, 4], [3, 4]], [[6, 2], [7, 2], [7, 2.5], [6, 2.6]], [[5, 1], [3, 2], [1, -2]]]
costs = [10,7,17]
budget = 6
epsilon = 1

# Construct visibility graph
graph = visibilityGraph(start, goal, obstacles, costs, budget)


copiedGraph = createCopiesOfGraph(graph, budget, epsilon)

def printGraph(graph):
    for vertex, neighbors in graph.vertices.items():
        print(f"Vertex {vertex}: Neighbors {neighbors}")
    
printGraph(graph)
printGraph(copiedGraph)

# Plot visibility graph
plotVisibilityGraph(start, goal, obstacles, graph)

