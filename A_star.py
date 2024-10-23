from pythonds.graphs import Graph, Vertex
import heapq
import networkx as nx
import matplotlib.pyplot as plt

def A_Star(graph, start, goal):
    toExplore = []
    heapq.heappush(toExplore, (start.getHeuristic(), id(start), start)) 

    Order = []

    cost = {start.getId(): 0}
    from_vertex = {start.getId(): None}

    while toExplore:
        f_n, _, curVertex = heapq.heappop(toExplore) 
        currentVertex = graph.getVertex(curVertex.getId())

        print(f"Exploring {currentVertex.getId()} with f(n) = {f_n}")

        Order.append(currentVertex)

        if currentVertex.getId() == goal.getId():
            print("Goal reached!")
            return reconstruct_path(from_vertex, goal), cost[goal.getId()], Order

        for neighbor in currentVertex.getConnections():
            
            edge_weight = currentVertex.getWeight(neighbor)
            temp_cost = cost[currentVertex.getId()] + edge_weight  

            print(f"Checking neighbor {neighbor.getId()} with edge weight = {edge_weight} and g(n) = {temp_cost}")

            if neighbor.getId() not in cost or temp_cost < cost[neighbor.getId()]:
                cost[neighbor.getId()] = temp_cost
                f_n = temp_cost + neighbor.getHeuristic()  # f(n) = g(n) + h(n)
                heapq.heappush(toExplore, (f_n, id(neighbor), neighbor))  
                from_vertex[neighbor.getId()] = currentVertex.getId()

    print("No path found")
    return None, None, Order

def reconstruct_path(from_vertex, goal):
    current_path = []
    current = goal.getId()
    while current is not None:
        current_path.append(current)
        current = from_vertex.get(current)
    current_path.reverse()
    return current_path

def heuristic(vertex, goalVertex):
    vertex.heuristic_val = abs(vertex.getX() - goalVertex.getX()) + abs(vertex.getY() - goalVertex.getY())
    print(f"The heuristic value of {vertex.getId()} is {vertex.getHeuristic()}")

def heuristic_goal(vertex):
    vertex.heuristic_val = 0
    return vertex

def undirected_connect(graph, one, two, weight):
    graph.addEdge(one.getId(), two.getId(), weight)  
    graph.addEdge(two.getId(), one.getId(), weight)  
    print(f"Connected {one.getId()} and {two.getId()} with weight {weight}")

def convert_to_nx_graph(pyThonds_graph):
    nx_graph = nx.Graph()
    
    for vertex in pyThonds_graph:
        nx_graph.add_node(vertex.getId(), heuristic=vertex.getHeuristic())

        for neighbor in vertex.getConnections():
            weight = vertex.getWeight(neighbor)
            nx_graph.add_edge(vertex.getId(), neighbor.getId(), weight=weight)

    return nx_graph

def visualize_search(order, graph, title, pos, path):
    plt.figure(figsize=(16, 10))  
    plt.title(title)

    exploredEdges = set()

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
        "L": (2, 1.5),
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

    for i, node in enumerate(order):
        plt.clf()
        plt.title(title)

        nx.draw(graph, pos, with_labels=True, node_color='green', font_weight='bold', node_size=700)

        for neighbor in node.getConnections():
            exploredEdges.add((node.getId(), neighbor.getId()))
            exploredEdges.add((neighbor.getId(), node.getId()))

        if i == len(order) - 1: 
            for j in range(len(path) - 1):
                nx.draw_networkx_edges(graph, pos, edgelist=[(path[j], path[j + 1])], edge_color='orange', width=3)
                plt.draw()
                plt.pause(2)

    plt.show()

def main():
    graph = Graph()

    # Create vertices with x and y values to be used get their heuristic values
    # 0 heuristic value = goal
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

    vertices = [A, B, C, D, E, F, G, H, I, J1, J2, K, L, M, N, O, P, Q, R, S, T, U]

    start_name = input("Enter start: ")
    goal_name = input("Enter goal: ")

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

    heuristic_goal(goal)  

    for vertex in vertices:
        heuristic(vertex, goal)

    for vertex in vertices:
        graph.addVertex(vertex.getId(), vertex.getX(), vertex.getY(), vertex.getHeuristic())

    # Add connections between vertices
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

    print("\n\nCommencing A_Star...")
    path, total_cost, order = A_Star(graph, start, goal)

    if path is None:
        print("No path found")
    else:
        print("Path found:", " -> ".join(path))
        print(f"Total Cost: {total_cost}")

        nx_graph = convert_to_nx_graph(graph)
        pos = nx.spring_layout(nx_graph, k=75, iterations=3000)
        visualize_search(order, nx_graph, "A* Search Visualization", pos, path)

if __name__ == '__main__':
    main()
