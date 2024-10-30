from pythonds.graphs import Graph, Vertex
import networkx as nx
import matplotlib.pyplot as plt

def undirected_connect(graph, one, two, weight=0):
    graph.addEdge(one.getId(), two.getId(), weight)
    graph.addEdge(two.getId(), one.getId(), weight)
    
def convert_to_nx_graph(pyThonds_graph):
    nx_graph = nx.Graph()
    
    for vertex in pyThonds_graph:
        nx_graph.add_node(vertex.getId(), heuristic=vertex.getHeuristic())

        for neighbor in vertex.getConnections():
            weight = vertex.getWeight(neighbor)
            nx_graph.add_edge(vertex.getId(), neighbor.getId(), weight=weight)

    return nx_graph
    
def visualize_dfs(graph, G, dfs_path, pos):
    plt.title = "DFS VISUALIZATION"
    
    # Add nodes and edges to the graph
    for vertex in graph.getVertices():
        G.add_node(vertex)

    for vertex in graph.getVertices():
        for neighbor in graph.getVertex(vertex).connectedTo:
            G.add_edge(vertex, neighbor.getId())

    # Initial plot setup
        pos = {
        "A": (11, 1),   
        "B": (10, 2),
        "C": (9, 2),
        "D": (8, 1),
        "E": (6, 2),
        "F": (5, 2),
        "G": (4, 2),
        "H": (4, 1),
        "I": (3, 2),
        "J1": (2, 2),
        "J2": (2, 1),
        "K": (1, 2),
        "L": (2.5, 1.5),
        "M": (1, 4),
        "N": (2, 4),
        "O": (3, 5),
        "P": (6, 5),
        "Q": (7, 5),
        "R": (8, 5),
        "S": (6, 6),
        "T": (11, 5),
        "U": (1, 1),
    }
    plt.figure(figsize=(16, 8))
    
    # Draw the graph with nodes and edges
    nx.draw(G, pos, with_labels=True, node_color='lightblue', edge_color='grey', node_size=500, font_size=10)
    
    # Iterate through the DFS path to show traversal
    for i in range(len(dfs_path) - 1):
        # Highlight current node and edge
        nx.draw(G, pos, with_labels=True, node_color='lightblue', edge_color='grey', node_size=500, font_size=10)
        
        # Highlight current DFS path
        path_edge = [(dfs_path[i], dfs_path[i+1])]
        nx.draw_networkx_edges(G, pos, edgelist=path_edge, edge_color='red', width=2)
        nx.draw_networkx_nodes(G, pos, nodelist=[dfs_path[i]], node_color='red', node_size=600)
        
        plt.pause(1)  # Pause to visualize the step (you can adjust the delay)
    
    # Final step - highlight the goal
    nx.draw_networkx_nodes(G, pos, nodelist=[dfs_path[-1]], node_color='green', node_size=600)
    
    plt.show()

def DFS(vertex, graph, visited=None, path=None, distance=0, goal=None, dfs_path = None):
    if visited is None:
        visited = set()
        
    if path is None:
        path = []
        
    if dfs_path is None:
        dfs_path = []

    vertex_id = vertex.getId()  # Get the vertex ID for comparisons

    # Check if vertex has already been visited
    if vertex_id in visited:
        return False  # Stop further exploration if already visited

    visited.add(vertex_id)  # Add the current vertex ID to the visited set
    path.append(vertex_id)  # Append the current vertex's ID to the path
    dfs_path.append(vertex_id)

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
            if DFS(next_vertex, graph, visited, path, distance + weight, goal, dfs_path):
                return True  # Stop further recursion once the goal is reached
    
    dfs_path.pop()
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
    O = Vertex("O", 11, 6)
    P = Vertex("P", 14, 6)
    Q = Vertex("Q", 15, 6)
    R = Vertex("R", 18, 6)
    S = Vertex("S", 14, 7)
    T = Vertex("T", 22, 6)
    U = Vertex("U", 2, 2)

    # Add vertices to graph
    vertices = [A, B, C, D, E, F, G, H, I, J1, J2, K, L, M, N, O, P, Q, R, S, T, U]
    
    start_name = input("Enter start node: ")
    goal_name = input("Enter goal node: ")

    start = None
    goal = None

    for vertex in vertices:
        if vertex.getId() == start_name:
            start = vertex
        if vertex.getId() == goal_name:
            goal = vertex


    if start is None or goal is None:
        print(f"Error: Start vertex '{start_name}' or goal vertex '{goal_name}' does not exist.")
        return

    for vertex in vertices:
        graph.addVertex(vertex.getId(), vertex.getX(), vertex.getY(), vertex.getHeuristic())

    # Connect vertices using the undirected_connect function
    undirected_connect(graph, U, K, 20)
    undirected_connect(graph, U, J2, 30)
    undirected_connect(graph, J2, J1, 45)
    undirected_connect(graph, K, J1, 10)
    undirected_connect(graph, K, M, 400)
    undirected_connect(graph, M, N, 25)
    undirected_connect(graph, J1, I, 10)
    undirected_connect(graph, J2, L, 35)
    undirected_connect(graph, J1, L, 20)
    undirected_connect(graph, L, I, 20)
    undirected_connect(graph, I, G, 15)
    undirected_connect(graph, L, H, 20)
    undirected_connect(graph, I, H, 25)
    undirected_connect(graph, G, H, 30)
    undirected_connect(graph, G, F, 20)
    undirected_connect(graph, H, F, 55)
    undirected_connect(graph, N, O, 120)
    undirected_connect(graph, F, O, 145)
    undirected_connect(graph, F, E, 10)
    undirected_connect(graph, O, E, 130)
    undirected_connect(graph, O, S, 40)
    undirected_connect(graph, O, P, 30)
    undirected_connect(graph, E, P, 145)
    undirected_connect(graph, E, D, 30)
    undirected_connect(graph, S, P, 60)
    undirected_connect(graph, D, P, 165)
    undirected_connect(graph, P, Q, 10)
    undirected_connect(graph, S, Q, 50)
    undirected_connect(graph, D, C, 90)
    undirected_connect(graph, Q, R, 25)
    undirected_connect(graph, R, C, 180)
    undirected_connect(graph, R, B, 140)
    undirected_connect(graph, C, B, 40)
    undirected_connect(graph, R, T, 20)
    undirected_connect(graph, B, T, 120)
    undirected_connect(graph, B, A, 15)
    undirected_connect(graph, A, T, 135)

    dfs_path = []
    
    
    if not DFS(start, graph, goal=goal, dfs_path=dfs_path):
        print(f"Goal {goal.getId()} not reachable from the start location.")
    else:
        nx_graph = convert_to_nx_graph(graph)
        pos = nx.spring_layout(nx_graph, k=75, iterations=3000)
        visualize_dfs(graph, nx_graph, dfs_path, pos)

if __name__ == '__main__':
    main()
