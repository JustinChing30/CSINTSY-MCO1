from pythonds.graphs import Graph, Vertex
import heapq


def A_Star(graph, start, goal):
    toExplore = []
    heapq.heappush(toExplore, (start.getHeuristic(), start))

    cost = {start: 0}
    from_vertex = {start: None}

    while toExplore:
        f_n, curVertex = heapq.heappop(toExplore)
        currentVertex = graph.getVertex(curVertex.getId())

        if currentVertex == goal:
            return reconstruct_path(from_vertex, goal)
        
        for neighbor in currentVertex.getConnections():
            temp_cost = cost[curVertex] + currentVertex.getWeight(neighbor)

            if neighbor not in cost or temp_cost < cost[neighbor]:
                cost[neighbor] = temp_cost
                f_n = temp_cost + neighbor.getHeuristic()  # f(n) = g(n) + h(n)
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

def undirected_connect(graph, one, two):
    graph.addEdge(one.getId(), two.getId())
    graph.addEdge(two.getId(), one.getId())

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
    U = Vertex("Fidel A. Reyes St.", 0)  # Goal

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

    # Capture the path found by A* algorithm
    path = A_Star(graph, B, J)

    # Print the path
    if path is not None:
        print("Path found:", " -> ".join(path))
    else:
        print("No path found")

if __name__ == '__main__':
    main()
