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

        Order.append(currentVertex)

        if currentVertex.getId() == goal.getId():
            return reconstruct_path(from_vertex, goal), Order

        for neighbor in currentVertex.getConnections():
            temp_cost = cost[currentVertex.getId()] + currentVertex.getWeight(neighbor)

            if neighbor.getId() not in cost or temp_cost < cost[neighbor.getId()]:
                cost[neighbor.getId()] = temp_cost
                f_n = temp_cost + neighbor.getHeuristic()  
                heapq.heappush(toExplore, (f_n, id(neighbor), neighbor))  
                from_vertex[neighbor.getId()] = currentVertex.getId()  

    return None, Order

def reconstruct_path(from_vertex, goal):
    current_path = []
    current = goal.getId()
    while current is not None:
        current_path.append(current)
        current = from_vertex.get(current)  
    current_path.reverse()
    return current_path

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

def convert_to_nx_graph(pyThonds_graph):
    nx_graph = nx.Graph() 

    for vertex in pyThonds_graph:
        nx_graph.add_node(vertex.getId(), heuristic=vertex.getHeuristic())

        for neighbor in vertex.getConnections():
            weight = vertex.getWeight(neighbor)  
            nx_graph.add_edge(vertex.getId(), neighbor.getId(), weight=weight)

    return nx_graph


def undirected_connect(graph, one, two, weight):
    graph.addEdge(one.getId(), two.getId(), weight)
    graph.addEdge(two.getId(), one.getId(), weight)

def main():
    graph = Graph()

    A = Vertex("A", 25) 
    B = Vertex("B", 5)
    C = Vertex("C", 3)
    D = Vertex("D", 1)
    E = Vertex("E", 2)
    F = Vertex("F", 50)
    G = Vertex("G", 11)
    H = Vertex("H", 1)
    I = Vertex("I", 19)
    J1 = Vertex("J1", 3)
    J2 = Vertex("J2", 2)
    K = Vertex("K", 3)
    L = Vertex("L", 9)
    M = Vertex("M", 15)
    N = Vertex("N", 3)
    O = Vertex("O", 16)
    P = Vertex("P", 4)
    Q = Vertex("Q", 16)
    R = Vertex("R", 5)
    S = Vertex("S", 1)
    T = Vertex("T", 20)
    U = Vertex("U", 0)  

    vertices = [A, B, C, D, E, F, G, H, I, J1, J2, K, L, M, N, O, P, Q, R, S, T, U]
    for vertex in vertices:
        graph.addVertex(vertex.getId(), vertex.getHeuristic())

    undirected_connect(graph, A, B, 2)
    undirected_connect(graph, A, T, 3)
    undirected_connect(graph, B, C, 4)
    undirected_connect(graph, B, R, 3)
    undirected_connect(graph, R, Q, 2)
    undirected_connect(graph, Q, P, 1)
    undirected_connect(graph, P, S, 5)
    undirected_connect(graph, P, O, 3)
    undirected_connect(graph, P, Q, 1)
    undirected_connect(graph, O, N, 8)
    undirected_connect(graph, N, M, 3)
    undirected_connect(graph, N, G, 4)
    undirected_connect(graph, G, E, 5)
    undirected_connect(graph, E, F, 3)
    undirected_connect(graph, F, D, 4)
    undirected_connect(graph, D, A, 3)
    undirected_connect(graph, G, I, 4)
    undirected_connect(graph, I, J1, 3)
    undirected_connect(graph, J1, J2, 2)
    undirected_connect(graph, J1, K, 3)
    undirected_connect(graph, K, U, 2)
    undirected_connect(graph, U, J2, 3)
    undirected_connect(graph, L, J2, 3)
    undirected_connect(graph, L, H, 3)
    undirected_connect(graph, H, F, 5)
    undirected_connect(graph, T, R, 5)
    undirected_connect(graph, T, B, 4)
    undirected_connect(graph, T, A, 3)

    path, order = A_Star(graph, T, K)

    nx_graph = convert_to_nx_graph(graph)
    pos = nx.spring_layout(nx_graph, k=1.5, iterations=350)
    visualize_search(order, nx_graph, "A* Search Visualization", pos, path)

    if path is not None:
        print("Path found:", " -> ".join(path))
    else:
        print("No path found")


if __name__ == '__main__':
    main()
