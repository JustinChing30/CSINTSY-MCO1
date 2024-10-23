from pythonds.graphs import Graph, Vertex
import heapq

def gbfs(graph, start, goal):
    toExplore = [] # List of nodes to explore
    explored = set() # Set of nodes already explored
    pathTrack = {start.getId(): [start]} # Stores path to goal

    heapq.heappush(toExplore, (start.getHeuristic(), start)) # push start vertex

    while toExplore: # Implicit boolean
        cur_heuristic, cur_vertex = heapq.heappop(toExplore) # Pops smallest value (priority queue)

        print(f"Exploring {cur_vertex.getId()} with a heuristic value of {cur_heuristic}")

        if cur_vertex.getId() == goal.getId(): # if goal found
            print("Path found!\n")
            totalCost = 0
            path = pathTrack[cur_vertex.getId()]

            print("\nCalculating total cost...")
            for i in range(len(path) - 1): # Get total path cost
                curr_vertex = path[i] # current vertex
                next_vertex = path[i + 1] # next vertex in path

                actual_neighbor = None # will keep next vertex
                for neighbor in curr_vertex.getConnections(): # scrolls through current vertex connections
                    if neighbor.getId() == next_vertex.getId(): # compares current connections names to next vertex name
                        actual_neighbor = neighbor # actual_neighbor references next_vertex basically
                        break

                weight = curr_vertex.getWeight(actual_neighbor) # gets weight from next_vertex
                print(f"Weight from {curr_vertex.getId()} to {next_vertex.getId()} is {weight}")
                totalCost += weight # adds weight to totalCost
                print(f"Current cost: {totalCost}")

            print("GBFS path to goal:", " -> ".join(v.getId() for v in path)) # Path to goal
            print(f"Total cost:  {totalCost}")
            return

        explored.add(cur_vertex.getId()) # Add current vertex to Explored

        current_vertex = graph.getVertex(cur_vertex.getId()) # Explore current vertex

        for connected in current_vertex.getConnections(): # checks connected nodes to current node

            if connected.getId() in explored: # if connected node is already in the explored set
                continue # basically this ignores that already explored node then goes to the next connected node

            if connected.getId() not in pathTrack: # if connected node is not in final track
                pathTrack[connected.getId()] = pathTrack[cur_vertex.getId()] + [connected]

            heapq.heappush(toExplore, (connected.getHeuristic(), connected)) # add connected node to queue

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
    A = Vertex("A", 21, 23)
    B = Vertex("B", 20, 3)
    C = Vertex("C", 19, 2)
    D = Vertex("D", 12, 1)
    E = Vertex("E", 11, 4)
    F = Vertex("F", 10, 4)
    G = Vertex("G", 8, 4)
    H = Vertex("H", 8, 0)
    I = Vertex("I'", 7, 4)
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

    print("\nCalculating heuristic values...")
    # Set the goal
    goal = heuristic_goal(goal) # change goal here

    # Calculate the heuristic_value per node in based on the goal
    for vertex in vertices:
        heuristic(vertex, goal)
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

    print("\n\nCommencing GBFS...")
    gbfs(graph, startNode, goal)

if __name__ == '__main__':
    main()