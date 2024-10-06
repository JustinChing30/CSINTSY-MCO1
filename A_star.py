from pythonds.graphs import Graph, Vertex
import heapq

# extends the Vertex class imported from pythonds to include a heuristic value
class VertexHeuristic(Vertex):
    def __init__(self, node_name, heuristic_val):  # Corrected constructor
        super().__init__(node_name)  # Call the constructor of the parent class (Vertex)
        self.heuristic_val = heuristic_val  # Store the heuristic value

def A_Star(graph, start, goal):
   toExplore = []
   
   heapq.heappush(toExplore, (start.heuristic_val, start))

   cost = {start: 0}
   from_vertex = {start: None}

   while toExplore:
       f_n, currentVertex = heapq.heappop(toExplore)

       if currentVertex == goal:
           return reconstruct_path(from_vertex, goal)
       
       for neighbor in currentVertex.getConnections(): 
         temp_val = cost[currentVertex] + currentVertex.getWeight(neighbor)
            
         if neighbor not in cost or temp_val < cost[neighbor]:
             cost[neighbor] = temp_val
             f_n = neighbor.heuristic_val + temp_val
             heapq.heappush(toExplore, (f_n, neighbor))
             from_vertex[neighbor] = currentVertex 
   return None


def reconstruct_path(from_vertex, goal):
    current_path = []  
    current = goal
    while current is not None:  
        current_path.append(current.getId())  
        current = from_vertex[current]  
    current_path.reverse()  
    return current_path

# undirected_connect Connects two nodes together (Undirected)
# graph is the graph
# one is first node
# two is second node
def undirected_connect(graph, one, two, weight):
    graph.addEdge(one, two, weight)
    graph.addEdge(two, one, weight)

def main():
    
    graph = Graph()

    A = VertexHeuristic("A", 10)
    B = VertexHeuristic("B", 8)
    C = VertexHeuristic("C", 5)
    D = VertexHeuristic("D", 7)
    E = VertexHeuristic("E", 3)
    F = VertexHeuristic("F", 0) 

    for vertex in [A, B, C, D, E, F]:
        graph.addVertex(vertex)

    undirected_connect(graph, A, B, 1)
    undirected_connect(graph, A, C, 3)
    undirected_connect(graph, B, D, 1)
    undirected_connect(graph, C, D, 1)
    undirected_connect(graph, C, E, 5)
    undirected_connect(graph, D, F, 2)
    undirected_connect(graph, E, F, 2)

    start = A
    goal = F
    path = A_Star(graph, start, goal)

    if path:
        print("Path found:", " -> ".join(path))
    else:
        print("No path found")


if __name__ == '__main__':
    main()
