from pythonds.graphs import Graph, Vertex
import heapq
import networkx as nx 
import matplotlib.pyplot as plt 

def A_Star(graph, start, goal):
    toExplore = []
    heapq.heappush(toExplore, (start.getHeuristic(), start))

    Order = []

    cost = {start.getId(): 0}
    from_vertex = {start.getId(): None}

    while toExplore:
        f_n, curVertex = heapq.heappop(toExplore)
        currentVertex = graph.getVertex(curVertex.getId())

        Order.append(currentVertex)

        if currentVertex.getId() == goal.getId():
            return reconstruct_path(from_vertex, goal), Order

        for neighbor in currentVertex.getConnections():
            temp_cost = cost[currentVertex.getId()] + currentVertex.getWeight(neighbor)

            if neighbor.getId() not in cost or temp_cost < cost[neighbor.getId()]:
                cost[neighbor.getId()] = temp_cost
                f_n = temp_cost + neighbor.getHeuristic()  
                heapq.heappush(toExplore, (f_n, neighbor))
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
    plt.figure(figsize=(12, 8))  
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


def undirected_connect(graph, one, two):
    graph.addEdge(one.getId(), two.getId())
    graph.addEdge(two.getId(), one.getId())


def main():
    graph = Graph()

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
    U = Vertex("Fidel A. Reyes St.", 0)  

    vertices = [A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U]
    for vertex in vertices:
        graph.addVertex(vertex.getId(), vertex.getHeuristic())

    undirected_connect(graph, B, E)
    undirected_connect(graph, E, G)
    undirected_connect(graph, G, J)
    undirected_connect(graph, E, A)
    undirected_connect(graph, A, J)
    undirected_connect(graph, B, D)
    undirected_connect(graph, D, A)
    undirected_connect(graph, D, F)
    undirected_connect(graph, F, J)

    
    path, order = A_Star(graph, B, J)

    nx_graph = convert_to_nx_graph(graph)
    pos = nx.spring_layout(nx_graph)
    visualize_search(order, nx_graph, "A* Search Visualization", pos, path)

    if path is not None:
        print("Path found:", " -> ".join(path))
    else:
        print("No path found")


if __name__ == '__main__':
    main()
