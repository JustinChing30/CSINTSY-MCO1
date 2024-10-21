from pythonds.graphs import Graph, Vertex

def undirected_connect(graph, one, two, weight=0):
    graph.addEdge(one.getId(), two.getId(), weight)
    graph.addEdge(two.getId(), one.getId(), weight)

def heuristic(vertex, goalVertex):
    vertex.heuristic_val = abs(vertex.getX() - goalVertex.getX()) + abs(vertex.getY() - goalVertex.getY())
    print(f"The heuristic value of {vertex.getId()} is {vertex.getHeuristic()}")

def heuristic_goal(vertex):
    vertex.heuristic_val = 0
    return vertex

def DFS(vertex, graph, visited=None, path=None, distance=0, goal=None):
    if visited is None:
        visited = set()
        
    if path is None:
        path = []

    vertex_id = vertex.getId()  # Get the vertex ID for comparisons

    # Check if vertex has already been visited
    if vertex_id in visited:
        return False  # Stop further exploration if already visited

    visited.add(vertex_id)  # Add the current vertex ID to the visited set
    path.append(vertex_id)  # Append the current vertex's ID to the path

    # Print current vertex and its heuristic
    print(f"Exploring {vertex_id} with heuristic: {vertex.getHeuristic()}")

    # Check if the goal is reached
    if vertex_id == goal.getId():  # Fixed goal comparison
        print(" -> ".join(path))
        print(f"Goal {goal.getId()} reached with total distance: {distance}")
        return True  # Stop the search once the goal is reached

    # Get all connected vertices and sort them alphabetically
    vertex_connect = list(graph.getVertex(vertex_id).connectedTo.items())
    vertex_connect.sort(key=lambda x: x[0].getId())  # Sort by the vertex ID

    # Recursively explore the neighbors
    for next_vertex, weight in vertex_connect:
        next_vertex_id = next_vertex.getId()  # Get the next vertex ID
        if next_vertex_id not in visited:  # Only explore unvisited neighbors
            # Recur with updated distance
            if DFS(next_vertex, graph, visited, path, distance + weight, goal):
                return True  # Stop further recursion once the goal is reached
    
    path.pop()  # Remove the vertex if it doesn't lead to the goal
    return False  # Return False if goal is not found

def main():
    graph = Graph()

    # Define the vertices with their heuristic values and positions
    A = Vertex("Arad", 2, 2)          # Heuristic: 366
    B = Vertex("Zerind", 4, 3)        # Heuristic: 374
    C = Vertex("Oradea", 5, 0)        # Heuristic: 380
    D = Vertex("Sibiu", 6, 1)         # Heuristic: 253
    E = Vertex("Timisoara", 8, 0)     # Heuristic: 329
    F = Vertex("Lugoj", 7, 4)         # Heuristic: 244
    G = Vertex("Mehadia", 10, 4)      # Heuristic: 241
    H = Vertex("Dobreta", 12, 1)      # Heuristic: 242
    I = Vertex("Craiova", 8, 4)       # Heuristic: 160
    J = Vertex("Rimnicu Vilcea", 14, 6) # Heuristic: 193
    K = Vertex("Pitesti", 18, 6)      # Heuristic: 100
    L = Vertex("Fagaras", 20, 3)      # Heuristic: 176
    M = Vertex("Bucharest", 22, 6)    # Heuristic: 0
    N = Vertex("Giurgiu", 14, 7)      # Heuristic: 77
    O = Vertex("Urziceni", 6, 6)       # Heuristic: 80
    P = Vertex("Vaslui", 1, 6)        # Heuristic: 199
    Q = Vertex("Iasi", 4, 6)          # Heuristic: 226
    R = Vertex("Neamt", 15, 6)        # Heuristic: 234
    S = Vertex("Hirsova", 19, 2)      # Heuristic: 151
    T = Vertex("Eforie", 11, 4)       # Heuristic: 161

    vertices = [A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T]

    print("\nCalculating heuristic values...")
    goal = heuristic_goal(M)

    for vertex in vertices:
        heuristic(vertex, goal)

    for vertex in vertices:
        graph.addVertex(vertex.getId(), vertex.getX(), vertex.getY(), vertex.getHeuristic())

    # Connect vertices using the undirected_connect function
    undirected_connect(graph, A, B, 75)  # Arad - Zerind
    undirected_connect(graph, A, D, 140) # Arad - Sibiu
    undirected_connect(graph, A, E, 118) # Arad - Timisoara
    undirected_connect(graph, B, C, 71)  # Zerind - Oradea
    undirected_connect(graph, C, D, 151) # Oradea - Sibiu
    undirected_connect(graph, E, F, 111) # Timisoara - Lugoj
    undirected_connect(graph, F, G, 70)  # Lugoj - Mehadia
    undirected_connect(graph, G, H, 75)  # Mehadia - Dobreta
    undirected_connect(graph, H, I, 120) # Dobreta - Craiova
    undirected_connect(graph, I, J, 146) # Craiova - Rimnicu Vilcea
    undirected_connect(graph, I, K, 138) # Craiova - Pitesti
    undirected_connect(graph, J, D, 80)  # Rimnicu Vilcea - Sibiu
    undirected_connect(graph, J, K, 97)  # Rimnicu Vilcea - Pitesti
    undirected_connect(graph, K, M, 101) # Pitesti - Bucharest
    undirected_connect(graph, L, D, 99)  # Fagaras - Sibiu
    undirected_connect(graph, L, M, 211) # Fagaras - Bucharest
    undirected_connect(graph, M, N, 90)  # Bucharest - Giurgiu
    undirected_connect(graph, M, O, 85)  # Bucharest - Urziceni
    undirected_connect(graph, O, S, 98)  # Urziceni - Hirsova
    undirected_connect(graph, S, T, 86)  # Hirsova - Eforie
    undirected_connect(graph, O, P, 142) # Urziceni - Vaslui
    undirected_connect(graph, P, Q, 92)  # Vaslui - Iasi
    undirected_connect(graph, Q, R, 87)  # Iasi - Neamt

    if not DFS(A, graph, goal=goal):
        print(f"Goal {goal.getId()} not reachable from the start location.")

if __name__ == '__main__':
    main()
