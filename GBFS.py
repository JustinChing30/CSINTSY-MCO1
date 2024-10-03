from pythonds.graphs import Graph, Vertex

# extends the Vertex class imported from pythonds to include a heuristic value
class VertexHeuristic(Vertex):
    def _init__(self, node_name, heuristic_val):
        super().__init__(node_name)
        self.heuristic_val = heuristic_val

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
    graph.addVertex(A)
    graph.addVertex(B)
    graph.addVertex(C)
    graph.addVertex(D)
    graph.addVertex(E)
    graph.addVertex(F)
    graph.addVertex(G)
    graph.addVertex(H)
    graph.addVertex(I)
    graph.addVertex(J)
    graph.addVertex(K)
    graph.addVertex(L)
    graph.addVertex(M)
    graph.addVertex(N)
    graph.addVertex(O)
    graph.addVertex(P)
    graph.addVertex(Q)
    graph.addVertex(R)
    graph.addVertex(S)
    graph.addVertex(T)
    graph.addVertex(U)

if __name__ == '__main__':
    main()