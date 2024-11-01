import heapq

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


def main(graph):
    start_name = input("Enter start: ")
    goal_name = input("Enter goal: ")

    start = None
    goal = None

    for vertex_id in graph.getVertices():  # Compares inputs to vertex IDs
        vertex = graph.getVertex(vertex_id)
        if vertex.getId() == start_name:
            start = vertex
        if vertex.getId() == goal_name:
            goal = vertex

    if start is None or goal is None:
        print(f"Error: Start vertex '{start_name}' or goal vertex '{goal_name}' does not exist.")
        return

    heuristic_goal(goal)

    for vertex_id in graph.getVertices():
        vertex = graph.getVertex(vertex_id)
        heuristic(vertex, goal)

    print("\n\nCommencing A_Star...")
    path, total_cost, order = A_Star(graph, start, goal)

    if path is None:
        print("No path found")
    else:
        print("Path found:", " -> ".join(path))
        print(f"Total Cost: {total_cost}")

        return path


