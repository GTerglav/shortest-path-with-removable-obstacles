import heapq
import helper


def dijkstra(graph, start, target):
    # Initialize distances to all vertices as infinity
    distances = {vertex: float("inf") for vertex in graph.vertices}
    distances[start] = 0

    # Initialize heap with start vertex
    heap = [(0, start)]

    # Track visited vertices
    visited = set()

    # Track previous vertices in the shortest path
    previous = {}

    while heap:
        # Pop the vertex with the smallest distance from heap
        currentDistance, currentVertex = heapq.heappop(heap)

        # If the vertex has already been visited, skip it
        if currentVertex in visited:
            continue

        # Mark the vertex as visited
        visited.add(currentVertex)

        # Update distances to neighbors
        for neighbor, (cost, dist) in graph.vertices[currentVertex].items():
            distance = currentDistance + dist
            try:
                a = distance < distances[neighbor]
            except:
                print(f"whoopsie: {neighbor} not in vertices")
            else:
                if a:
                    distances[neighbor] = distance
                    previous[neighbor] = currentVertex  # Update previous vertex
                    heapq.heappush(heap, (distance, neighbor))

    # If target vertex is not reachable, return None
    if target not in distances or distances[target] == float("inf"):
        print("Vertex not reachable")
        return None

    # Reconstruct shortest path
    path = [target]
    while path[-1] != start:
        path.append(previous[path[-1]])
    path.reverse()

    return path
