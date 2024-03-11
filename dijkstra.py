import heapq
 

class Graph():
 
    def __init__(self, vertices):
        self.V = vertices
        self.graph = [[0 for column in range(vertices)]
                      for row in range(vertices)]
 
    #takes the dictionary where each vertex has its previous vertex and then computes path from source to sink
    def fullPath(self, previousVertices, source, sink):
        path = [sink]
        a = previousVertices[sink]
        while a != source:
            path.append(a)
            a = previousVertices[a]
        path.append(source)
        path.reverse()
        return path
    
    def dijkstra(self, source, sink):

        distances = [1e7] * self.V
        distances[source] = 0
        visitedVertices =  [False] * self.V
        previousVertices = {}
        queue = []
        heapq.heapify(queue)
        heapq.heappush(queue, [distances[source], source])
        currentVertex = source


        while len(queue) != 0:
            #remove current vertex from priorityqueue
            heapq.heappop(queue)
            #check neighbors that are not already visited
            for i in range(self.V):
                #check if already visited
                if not visitedVertices[i]:
                    #check if vertices connected in graph
                    if self.graph[currentVertex][i] > 0:
                        #here we add distance from current neighbor to current vertex and check if its smaller than already know distance then include into or queue
                        alt = distances[currentVertex] + self.graph[currentVertex][i]
                        if alt < distances[i]:
                            distances[i] = alt
                            previousVertices[i] = currentVertex
                            heapq.heappush(queue, [distances[i], i])
            #mark current vertex as visited
            visitedVertices[currentVertex] = True
            #take first element from queue to be next current vertex
            if len(queue) != 0:
                currentVertex = heapq.nsmallest(1,queue)[0][1]
                if currentVertex == sink:
                    return distances[sink], self.fullPath(previousVertices, source, sink)
        return distances, previousVertices
            
g = Graph(9)
g.graph = [[0, 4, 0, 0, 0, 0, 0, 8, 0],
           [4, 0, 8, 0, 0, 0, 0, 11, 0],
           [0, 8, 0, 7, 0, 4, 0, 0, 2],
           [0, 0, 7, 0, 9, 14, 0, 0, 0],
           [0, 0, 0, 9, 0, 10, 0, 0, 0],
           [0, 0, 4, 14, 10, 0, 2, 0, 0],
           [0, 0, 0, 0, 0, 2, 0, 1, 6],
           [8, 11, 0, 0, 0, 0, 1, 0, 7],
           [0, 0, 2, 0, 0, 0, 6, 7, 0]
           ]
 
print("Dijkstra:",g.dijkstra(0,4))



