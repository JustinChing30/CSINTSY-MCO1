from pythonds.graphs import Graph, Vertex
import networkx as nx
import matplotlib.pyplot as plt
import DFS
import GBFS
import A_star

def undirected_connect(graph, one, two, weight=0):
    graph.addEdge(one.getId(), two.getId(), weight)
    graph.addEdge(two.getId(), one.getId(), weight)

    one.addNeighbor(two, weight)
    two.addNeighbor(one, weight)

def initialize_graph():
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

    return graph

def add_vertex(graph):
    new_id = input("Input vertex ID to add: ")

    # loop until user inputs valid x and y values
    while True:
        try:
            x = int(input("Enter x-coordinate: "))
            y = int(input("Enter y-coordinate: "))

            # Check if a vertex already exists at (x, y)
            if any(graph.getVertex(v).getX() == x and graph.getVertex(v).getY() == y for v in graph.getVertices()):
                print(f"A vertex already exists at ({x}, {y}). Please enter different coordinates.")
            else:
                break  # Valid and unique coordinates, break loop
        except ValueError:
            print("Invalid input. Please enter integer values for x and y coordinates.")

    vertex = Vertex(new_id, x, y)
    graph.addVertex(vertex.getId(), vertex.getX(), vertex.getY(), vertex.getHeuristic())
    print(f"Vertex {new_id} added at ({x}, {y}).")

def add_edge(graph):
    start_id = input("Enter start vertex ID: ")
    end_id = input("Enter end vertex ID: ")

    # loop until user inputs valid integer for the weight
    while True:
        try:
            weight = int(input("Enter edge weight: "))
            break

        except ValueError:
            print("Invalid input. Please enter an integer for the weight.")

    start = graph.getVertex(start_id)
    end = graph.getVertex(end_id)

    if start and end:
        undirected_connect(graph, start, end, weight)
        print(f"Edge between {start_id} and {end_id} added.")
    else:
        print("One or both vertices not found.")

def remove_vertex(graph):
    vertex_to_remove = input("Enter vertex id to remove: ")

    # Check if the vertex exists in the graph
    if vertex_to_remove:
        # Create a new empty graph
        new_graph = Graph()

        # Copy all vertices except the one to be removed
        for vertex in graph.getVertices():
            if vertex != vertex_to_remove:
                new_graph.addVertex(vertex, graph.getVertex(vertex).getX(), graph.getVertex(vertex).getY(),
                                    graph.getVertex(vertex).getHeuristic())

        # Copy all edges except those involving the removed vertex
        for vertex in graph.getVertices():
            if vertex != vertex_to_remove:
                for neighbor in graph.getVertex(vertex).getConnections():
                    if neighbor.getId() != vertex_to_remove:
                        weight = graph.getVertex(vertex).getWeight(neighbor)
                        undirected_connect(new_graph, graph.getVertex(vertex), neighbor, weight)

        print(f"Vertex {vertex_to_remove} has been removed.")
        return new_graph  # Return the new graph without the vertex to be removed
    else:
        print("Vertex not found in the graph.")
        return graph  # Return the original graph if vertex does not exist

def convert_to_nx_graph(graph):
    nx_graph = nx.Graph()
    pos = {}

    for vertex in graph:
        # Add node with heuristic data
        nx_graph.add_node(vertex.getId(), heuristic=vertex.getHeuristic())

        # Assume the vertex has x and y attributes for position
        pos[vertex.getId()] = (vertex.getX(), vertex.getY())

        # Add edges with weights
        for neighbor in vertex.getConnections():
            weight = vertex.getWeight(neighbor)
            nx_graph.add_edge(vertex.getId(), neighbor.getId(), weight=weight)

    return nx_graph, pos

def update_graph(graph):
    while True:
        print("\nWhat will you update in the graph?")
        print("1. Add Vertex")
        print("2. Add Edge")
        print("3. Remove Vertex")
        print("4. Back to Main Menu")

        choice = input("\nEnter the number of your choice: ")

        if choice == '1':
            add_vertex(graph)
        elif choice == '2':
            add_edge(graph)
        elif choice == '3':
            graph = remove_vertex(graph)
        elif choice == '4':
            print("\nReturning to main menu...\n")
            break  # Exit the loop and return to main menu
        else:
            print("Invalid choice. Please select 1, 2, 3, or 4.")

    return graph


def visualize_graph(graph):
    nx_graph, pos = convert_to_nx_graph(graph)

    # Window size
    plt.figure(figsize=(16, 8))
    plt.title("Graph Visualization")

    # Draw the graph
    nx.draw(nx_graph, pos, with_labels=True, node_color="orange", edge_color="grey", node_size=500, font_size=10,
            font_weight="bold")

    plt.show()

def visualize_path(graph, path, algorithm_name):
    nx_graph, pos = convert_to_nx_graph(graph)

    # Window size
    plt.figure(figsize=(16, 8))
    plt.title(f"{algorithm_name} Visualization")

    # Optimal path visualization
    for i in range(len(path) - 1):
        # Base graph is drawn
        nx.draw(nx_graph, pos, with_labels=True, node_color="orange", edge_color="grey", node_size=500, font_size=10,
                font_weight="bold")

        # Note to do the stuff from current node to the next node
        path_edge = [(path[i], path[i + 1])]

        # Highlights edges
        nx.draw_networkx_edges(nx_graph, pos, edgelist=path_edge, edge_color="green", width=2)

        # Highlights nodes
        nx.draw_networkx_nodes(nx_graph, pos, nodelist=[path[i]], node_color="green", node_size=700)

        # Pause for a bit
        plt.pause(0.5)

    # Highlights goal
    nx.draw_networkx_nodes(nx_graph, pos, nodelist=[path[-1]], node_color="yellow", node_size=700)

    # Prevents visualization from instantly closing
    plt.show()

def run_algorithm(graph):
    while True:  # Loop to allow repeated algorithm selection
        path = []  # Reset path variable every time menu is opened

        print("\nSelect algorithm to run:")
        print("1. Depth-First Search")
        print("2. Greedy Best-First Search")
        print("3. A* Search")
        print("4. Back to Main Menu")  # Option to exit the program

        choice = input("\nEnter the number of your choice: ")

        if choice == '1':
            path = DFS.main(graph)  # Call the main function of DFS.py
            visualize_path(graph, path, "DFS")
        elif choice == '2':
            path = GBFS.main(graph)  # Call the main function of GBFS.py
            visualize_path(graph, path, "GBFS")
        elif choice == '3':
            path = A_star.main(graph)  # Call the main function of A_star.py
            visualize_path(graph, path, "A* Search")
        elif choice == '4':
            print("\nReturning to main menu...\n")
            break  # Exit the loop and end the program
        else:
            print("Invalid choice. Please select 1, 2, 3, or 4.")

def main():

    graph = initialize_graph()

    while True:  # Loop to allow repeated algorithm selection
        print("\nGood day! What will you do:")
        print("1. Run Algorithm")
        print("2. Update Graph")
        print("3. Visualize Graph")
        print("4. Close Program")  # Option to exit the program

        choice = input("\nEnter the number of your choice: ")

        if choice == '1':
            run_algorithm(graph)
        elif choice == '2':
            graph = update_graph(graph)
        elif choice == '3':
            visualize_graph(graph)
        elif choice == '4':
            print("Exiting the program...")
            break  # Exit the loop and end the program
        else:
            print("Invalid choice. Please select 1, 2, 3, or 4.")


if __name__ == '__main__':
    main()