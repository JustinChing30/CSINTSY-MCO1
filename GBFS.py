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
            forVisualize = []

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

            for v in path:
                forVisualize.append(v.getId())

            return forVisualize

        explored.add(cur_vertex.getId()) # Add current vertex to Explored

        current_vertex = graph.getVertex(cur_vertex.getId()) # Explore current vertex

        for connected in current_vertex.getConnections(): # checks connected nodes to current node

            if connected.getId() in explored: # if connected node is already in the explored set
                continue # basically this ignores that already explored node then goes to the next connected node

            if connected.getId() not in pathTrack: # if connected node is not in final track
                pathTrack[connected.getId()] = pathTrack[cur_vertex.getId()] + [connected]

            heapq.heappush(toExplore, (connected.getHeuristic(), connected)) # add connected node to queue

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

# Main function
def main(graph):

    start_name = input("Enter start node: ")  # Gets input for start node
    goal_name = input("Enter goal node: ")  # Gets input for goal node

    start = None
    goal = None

    for vertex_id in graph.getVertices():  # Compares inputs to vertex IDs
        vertex = graph.getVertex(vertex_id)
        if vertex.getId() == start_name:
            start = vertex
        if vertex.getId() == goal_name:
            goal = vertex

    if start is None or goal is None:
        print(f"Error: start vertex '{start_name}' or goal vertex '{goal_name}' does not exist.")
        return

    print("\nCalculating heuristic values...")
    # Set the goal
    heuristic_goal(goal)  # change goal here

    # Calculate the heuristic_value per node in based on the goal
    for vertex_id in graph.getVertices():
        vertex = graph.getVertex(vertex_id)
        heuristic(vertex, goal)

    print("\n\nCommencing GBFS...")
    path = gbfs(graph, start, goal)

    if path is None:
        print("No path found")
    else:
        return path
