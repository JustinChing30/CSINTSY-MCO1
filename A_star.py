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
            return reconstruct_path(from_vertex, goal), Order

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
    return None, Order

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
    plt.figure(figsize=(14, 10))  
    plt.title(title)

    exploredEdges = set()

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

    A = Vertex("A", 21, 3)
    B = Vertex("B", 20, 3)
    C = Vertex("C", 19, 2)
    D = Vertex("D", 12, 1)
    E = Vertex("E", 11, 4)
    F = Vertex("F", 10, 4)
    G = Vertex("G", 8, 4)
    H = Vertex("H", 8, 0)
    I = Vertex("I", 7, 4)
    J = Vertex("J1", 5, 3)
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
    J2 = Vertex("J2", 5, 0)

    vertices = [A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, J2]

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

    connections = [
        (A, B, 3), (A, C, 2), (B, C, 4), (B, T, 3), (C, R, 4),
        (T, R, 5), (R, Q, 2), (Q, P, 1), (P, S, 5), (P, O, 3),
        (S, Q, 4), (O, N, 3), (N, G, 2), (G, F, 5), (F, E, 3),
        (E, H, 5), (H, L, 3), (L, J, 2), (J, K, 3), (K, U, 2),
        (J, J2, 3), (J2, L, 2), (U, K, 4), (O, G, 3), (N, M, 3),
        (M, K, 3), (O, G, 3), (G, I, 2), (I, F, 4), (D, G, 4),
    ]

    for conn in connections:
        undirected_connect(graph, conn[0], conn[1], conn[2])

    path, order = A_Star(graph, start, goal)

    if path is None:
        print("No path found")
    else:
        print("Path found:", " -> ".join(path))

        nx_graph = convert_to_nx_graph(graph)
        pos = nx.spring_layout(nx_graph, k=1.5, iterations=350)
        visualize_search(order, nx_graph, "A* Search Visualization", pos, path)

if __name__ == '__main__':
    main()
