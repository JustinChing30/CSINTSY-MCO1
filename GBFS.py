from pythonds.graphs import Graph, Vertex
import heapq

def gbfs(graph, start, goal):
    toExplore = [] # List of nodes to explore
    explored = set() # Set of nodes already explored
    pathTrack = {start.getId(): [start]} # Stores path to goal

    heapq.heappush(toExplore, (start.getHeuristic(), start))

    while toExplore: # Implicit boolean
        cur_heuristic, cur_vertex = heapq.heappop(toExplore) # Pops smallest value

        print(f"Exploring: {cur_vertex.getId()} with heuristic {cur_heuristic}")

        if cur_vertex.getId() == goal.getId():
            print("Path to goal:", " -> ".join(v.getId() for v in pathTrack[cur_vertex.getId()])) # Path to goal
            return

        explored.add(cur_vertex.getId())

        current_vertex = graph.getVertex(cur_vertex.getId())

        for connected in current_vertex.getConnections(): # checks connected nodes to current node

            if connected.getId() in explored:
                continue # basically this ignores that already explored node

            if connected.getId() not in pathTrack:
                pathTrack[connected.getId()] = pathTrack[cur_vertex.getId()] + [connected]

            heapq.heappush(toExplore, (connected.getHeuristic(), connected))

# undirected_connect Connects two nodes together (Undirected)
# graph is the graph
# one is first node
# two is second node
def undirected_connect(graph, one, two):
    graph.addEdge(one.getId(), two.getId())
    graph.addEdge(two.getId(), one.getId())

# heuristic Calculates the heuristic value for a node
# vertex is the first node
# goalVertex is the goal node
def heuristic(vertex, goalVertex):
    vertex.heuristic_val = abs(vertex.getX() - goalVertex.getX()) + abs(vertex.getY() - goalVertex.getY())
    print(vertex.getId(), vertex.getHeuristic())

# heuristic_goal changes the heuristic_val of the goal node to 0
# vertex is the goal node
# Return type: vertex
def heuristic_goal(vertex):
    vertex.heuristic_val = 0
    return vertex

# Main function
def main():
    # Create graph
    graph = Graph()

    # Create vertices with x and y values to be used get their heuristic values
    # 0 heuristic value = goal
    A = Vertex("University Mall", 2, 2)
    B = Vertex("McDonald's", 20, 3)
    C = Vertex("Perico's", 19, 2)
    D = Vertex("Bloemen Hall", 12, 1)
    E = Vertex("W.H. Taft Residence", 11, 4)
    F = Vertex("EGI Taft", 10, 4)
    G = Vertex("Castro Street", 8, 4)
    H = Vertex("Agno Food Court", 8, 0)
    I = Vertex("One Archers'", 7, 4)
    J = Vertex("La Casita", 5, 0)
    K = Vertex("Green Mall", 4, 3)
    L = Vertex("Green Court", 6, 1)
    M = Vertex("Sherwood", 4, 6)
    N = Vertex("Jollibee", 6, 6)
    O = Vertex("Dagonoy St.", 1, 6)
    P = Vertex("Burgundy", 14, 6)
    Q = Vertex("Estrada St.", 15, 6)
    R = Vertex("D'Student's Place", 18, 6)
    S = Vertex("Leon Guinto St.", 14, 7)
    T = Vertex("P. Ocampo St.", 22, 6)
    U = Vertex("Fidel A. Reyes St.", 2, 2)

    # Add vertices to graph
    vertices = [A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U]

    # Set the goal
    goal = heuristic_goal(D) # change goal here

    # Calculate the heuristic_value per node in based on the goal
    for vertex in vertices:
        heuristic(vertex, goal)

    for vertex in vertices:
        graph.addVertex(vertex.getId(), vertex.getX(), vertex.getY(), vertex.getHeuristic())

    # Add connections between vertices
    undirected_connect(graph, A, Q)
    undirected_connect(graph, A, C)
    undirected_connect(graph, A, E)
    undirected_connect(graph, C, E)
    undirected_connect(graph, C, goal)
    undirected_connect(graph, Q, goal)
    undirected_connect(graph, E, goal)

    gbfs(graph, A, goal)

if __name__ == '__main__':
    main()