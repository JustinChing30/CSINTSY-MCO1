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
    # print(f"Exploring {vertex_id} with heuristic: {vertex.getHeuristic()}")
    print(f"Exploring {vertex_id}")

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
    A = Vertex("A", 21, 3)
    B = Vertex("B", 20, 3)
    C = Vertex("C", 19, 2)
    D = Vertex("D", 12, 1)
    E = Vertex("E", 11, 4)
    F = Vertex("F", 10, 4)
    G = Vertex("G", 8, 4)
    H = Vertex("H", 8, 0)
    I = Vertex("I", 7, 4)
    J1 = Vertex("J1", 5, 3)
    J2 = Vertex("J2", 5, 0)
    K = Vertex("K", 4, 3)
    L = Vertex("L", 6, 1)
    M = Vertex("M", 4, 6)
    N = Vertex("N", 6, 6)
    O = Vertex("O", 1, 6)
    P = Vertex("P", 14, 6)
    Q = Vertex("Q", 15, 6)
    R = Vertex("R", 18, 6)
    S = Vertex("S", 14, 7)
    T = Vertex("T", 22, 6)
    U = Vertex("U", 2, 2)

    # Add vertices to graph
    vertices = [A, B, C, D, E, F, G, H, I, J1, J2, K, L, M, N, O, P, Q, R, S, T, U]
    
    # start_name = input("Enter start: ")
    # goal_name = input("Enter goal: ")

    # start = None
    # goal = None

    # for vertex in vertices:
    #     if vertex.getId() == start_name:
    #         start = vertex
    #     if vertex.getId() == goal_name:
    #         goal = vertex


    # if start is None or goal is None:
    #     print(f"Error: Start vertex '{start_name}' or goal vertex '{goal_name}' does not exist.")
    #     return

    for vertex in vertices:
        graph.addVertex(vertex.getId(), vertex.getX(), vertex.getY(), vertex.getHeuristic())

    # Connect vertices using the undirected_connect function
    undirected_connect(graph, A, B, 15)
    undirected_connect(graph, A, T, 135)
    undirected_connect(graph, B, T, 120)
    undirected_connect(graph, B, R, 140)
    undirected_connect(graph, B, C, 40)
    undirected_connect(graph, C, R, 180)
    undirected_connect(graph, C, D, 90)
    undirected_connect(graph, R, T, 20)
    undirected_connect(graph, R, Q, 25)
    undirected_connect(graph, Q, S, 50)
    undirected_connect(graph, Q, P, 10)
    undirected_connect(graph, S, O, 40)  
    undirected_connect(graph, S, P, 60)
    undirected_connect(graph, O, P, 30)
    undirected_connect(graph, P, D, 165) 
    undirected_connect(graph, P, E, 145) 
    undirected_connect(graph, D, E, 30)
    undirected_connect(graph, O, E, 130)  
    undirected_connect(graph, O, F, 145)  
    undirected_connect(graph, F, E, 10) 
    undirected_connect(graph, O, N, 120) 
    undirected_connect(graph, F, H, 55) 
    undirected_connect(graph, F, G, 20) 
    undirected_connect(graph, H, G, 30)
    undirected_connect(graph, I, G, 15) 
    undirected_connect(graph, H, I, 25) 
    undirected_connect(graph, H, L, 20) 
    undirected_connect(graph, I, L, 20) 
    undirected_connect(graph, I, J1, 10) 
    undirected_connect(graph, L, J1, 20) 
    undirected_connect(graph, L, J2, 35) 
    undirected_connect(graph, J1, J2, 45) 
    undirected_connect(graph, J1, K, 10) 
    undirected_connect(graph, J2, U, 30) 
    undirected_connect(graph, U, K, 20) 
    undirected_connect(graph, M, K, 400) 
    undirected_connect(graph, M, N, 25) 
    
    start = D
    goal = S

    if not DFS(start, graph, goal=goal):
        print(f"Goal {goal.getId()} not reachable from the start location.")

if __name__ == '__main__':
    main()
