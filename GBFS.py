from pythonds.graphs import Graph, Vertex
import heapq

def gbfs(graph, start, goal):
    toExplore = [] # List of nodes to explore
    explored = set() # Set of nodes already explored
    pathTrack = {start.getId(): [start]} # Stores path to goal

    heapq.heappush(toExplore, (start.getHeuristic(), start))

    while toExplore: # Implicit boolean
        cur_heuristic, cur_vertex = heapq.heappop(toExplore) # Pops smallest value

        if cur_vertex.getId() == goal.getId():
            print("Path to goal:", " -> ".join(v.getId() for v in pathTrack[cur_vertex.getId()])) # Path to goal
            break

        explored.add(cur_vertex.getId())

        current_vertex = graph.getVertex(cur_vertex.getId())

        for connected in current_vertex.getConnections(): # checks connected nodes to current node

            if connected.getId() in explored:
                continue # basically this ignores that already explored node

            if connected.getId() not in pathTrack:
                pathTrack[connected.getId()] = pathTrack[cur_vertex.getId()] + [connected]

            heapq.heappush(toExplore, (connected.getHeuristic(), connected))

# undirected_connect Connects two nodes together (Undirected)
# graph is the graph
# one is first node
# two is second node
def undirected_connect(graph, one, two):
    graph.addEdge(one.getId(), two.getId())
    graph.addEdge(two.getId(), one.getId())

# Main function
def main():
    # Create graph
    graph = Graph()

    # Create vertices with heuristic values
    # 0 heuristic value = goal
    A = Vertex("University Mall", 25)
    B = Vertex("McDonald's", 5)
    C = Vertex("Perico's", 3)
    D = Vertex("Bloemen Hall", 1)
    E = Vertex("W.H. Taft Residence", 2)
    F = Vertex("EGI Taft", 50)
    G = Vertex("Castro Street", 11)
    H = Vertex("Agno Food Court", 1)
    I = Vertex("One Archers'", 19)
    J = Vertex("La Casita", 0)
    K = Vertex("Green Mall", 3)
    L = Vertex("Green Court", 9)
    M = Vertex("Sherwood", 15)
    N = Vertex("Jollibee", 3)
    O = Vertex("Dagonoy St.", 16)
    P = Vertex("Burgundy", 4)
    Q = Vertex("Estrada St.", 16)
    R = Vertex("D'Student's Place", 5)
    S = Vertex("Leon Guinto St.", 1)
    T = Vertex("P. Ocampo St.", 20)
    U = Vertex("Fidel A. Reyes St.", 0) # Goal

    # Add vertices to graph
    vertices = [A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U]
    for vertex in vertices:
        graph.addVertex(vertex.getId(), vertex.getHeuristic())

    # Add connections between vertices
    undirected_connect(graph, B, E)
    undirected_connect(graph, E, G)
    undirected_connect(graph, G, J)
    undirected_connect(graph, E, A)
    undirected_connect(graph, A, J)
    undirected_connect(graph, B, D)
    undirected_connect(graph, D, A)
    undirected_connect(graph, D, F)
    undirected_connect(graph, F, J)

    gbfs(graph, B, J)

if __name__ == '__main__':
    main()