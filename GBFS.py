from pythonds.graphs import Graph, Vertex
import heapq

def gbfs(graph, start, goal):
    toExplore = [] # List of nodes to explore
    explored = set() # Set of nodes already explored
    pathTrack = {start.getId(): [start]} # Stores path to goal

    heapq.heappush(toExplore, (start.getHeuristic(), start))

    while toExplore: # Implicit boolean
        cur_heuristic, cur_vertex = heapq.heappop(toExplore) # Pops smallest value (priority queue)

        print(f"Exploring {cur_vertex.getId()} with a heuristic value of {cur_heuristic}")

        if cur_vertex.getId() == goal.getId():
            totalCost = 0
            path = pathTrack[cur_vertex.getId()]

            print("\nCalculating total cost...")
            for i in range(len(path) - 1): # Get total path cost
                curr_vertex = path[i] # current vertex
                next_vertex = path[i + 1] # next vertex in path

                print(f"Current vertex: {curr_vertex.getId()}")
                print(f"Next vertex: {next_vertex.getId()}")
                print(f"Connections of {curr_vertex.getId()}: {[v.getId() for v in curr_vertex.getConnections()]}")

                if next_vertex not in curr_vertex.getConnections():
                    print(f"{next_vertex.getId()} is not a neighbor of {curr_vertex.getId()}")
                    continue  # Skip if not connected

                weight = curr_vertex.getWeight(next_vertex)
                print(f"Weight from {curr_vertex.getId()} to {next_vertex.getId()} is {curr_vertex.getWeight(next_vertex)}")
                totalCost += weight

            print("\nPath found!\n")
            print("GBFS path to goal:", " -> ".join(v.getId() for v in path)) # Path to goal
            print(f"Total cost:  {totalCost}")
            return

        explored.add(cur_vertex.getId()) # Add current vertex to Explored

        current_vertex = graph.getVertex(cur_vertex.getId()) # Explore current vertex

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
def undirected_connect(graph, one, two, weight):
    graph.addEdge(one.getId(), two.getId(), weight)
    graph.addEdge(two.getId(), one.getId(), weight)

    one.addNeighbor(two, weight)
    two.addNeighbor(one, weight)

# heuristic Calculates the heuristic value for a node
# vertex is the first node
# goalVertex is the goal node
def heuristic(vertex, goalVertex):
    vertex.heuristic_val = abs(vertex.getX() - goalVertex.getX()) + abs(vertex.getY() - goalVertex.getY())
    print(f"The heuristic value of {vertex.getId()} is {vertex.getHeuristic()}")

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

    print("\nCalculating heuristic values...")
    # Set the goal
    goal = heuristic_goal(E) # change goal here

    # Calculate the heuristic_value per node in based on the goal
    for vertex in vertices:
        heuristic(vertex, goal)

    for vertex in vertices:
        graph.addVertex(vertex.getId(), vertex.getX(), vertex.getY(), vertex.getHeuristic())

    # Add connections between vertices
    undirected_connect(graph, A, B, 50)
    undirected_connect(graph, B, C, 5)
    undirected_connect(graph, C, D, 7)
    undirected_connect(graph, D, goal, 9)

    print("\n\n\nCommencing GBFS...")
    gbfs(graph, A, goal)

if __name__ == '__main__':
    main()