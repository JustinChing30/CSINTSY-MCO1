from pythonds.graphs import Graph, Vertex
import heapq

# extends the Vertex class imported from pythonds to include a heuristic value
class VertexHeuristic(Vertex):
    def __init__(self, node_name, heuristic_val):
        super().__init__(node_name)
        self.heuristic_val = heuristic_val

def gbfs(start, goal):
    toExplore = [] # List of nodes to explore
    explored = set() # Set of nodes already explored
    pathTrack = [] # Stores path to goal

    heapq.heappush(toExplore, (start.heuristic_val, start))
    pathTrack.append([start]) # Initialize pathTrack

    while toExplore: # Implicit boolean
        cur_heuristic, cur_vertex = heapq.heappop(toExplore) # Pops smallest value

        if cur_vertex == goal:
            print("Path to goal: ", pathTrack[-1]) # Path to goal
            break

        explored.add(cur_vertex)

        for connected in cur_vertex.getConnections(): # checks connected nodes to current node

            if connected in explored:
                continue # basically this ignores that already explored node

            new_path = pathTrack[-1] + [connected] # Concatenation for path
            pathTrack.append(new_path) # Stores a bunch of paths, but we only take the last one anyway

            heapq.heappush(toExplore, (connected.heuristic_val, connected))

# undirected_connect Connects two nodes together (Undirected)
# graph is the graph
# one is first node
# two is second node
def undirected_connect(graph, one, two):
    graph.addEdge(one, two)
    graph.addEdge(two, one)

# Main function
def main():
    # Create graph
    graph = Graph()

    # Create vertices with heuristic values
    # 0 heuristic value = goal
    A = VertexHeuristic("University Mall", 3)
    B = VertexHeuristic("McDonald's", 7)
    C = VertexHeuristic("Perico's", 10)
    D = VertexHeuristic("Bloemen Hall", 2)
    E = VertexHeuristic("W.H. Taft Residence", 2)
    F = VertexHeuristic("EGI Taft", 5)
    G = VertexHeuristic("Castro Street", 8)
    H = VertexHeuristic("Agno Food Court", 1)
    I = VertexHeuristic("One Archers'", 19)
    J = VertexHeuristic("La Casita", 11)
    K = VertexHeuristic("Green Mall", 3)
    L = VertexHeuristic("Green Court", 9)
    M = VertexHeuristic("Sherwood", 15)
    N = VertexHeuristic("Jollibee", 3)
    O = VertexHeuristic("Dagonoy St.", 16)
    P = VertexHeuristic("Burgundy", 4)
    Q = VertexHeuristic("Estrada St.", 16)
    R = VertexHeuristic("D'Student's Place", 5)
    S = VertexHeuristic("Leon Guinto St.", 1)
    T = VertexHeuristic("P. Ocampo St.", 20)
    U = VertexHeuristic("Fidel A. Reyes St.", 0) # Goal

    # Add vertices to graph
    vertices = [A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U]
    for vertex in vertices:
        graph.addVertex(vertex)

    # Add connections between vertices

    gbfs(A, U)

if __name__ == '__main__':
    main()