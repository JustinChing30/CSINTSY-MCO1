def DFS(vertex, graph, visited=None, path=None, distance=0, goal=None, dfs_path=None):
    if visited is None:
        visited = set()

    if path is None:
        path = []

    if dfs_path is None:
        dfs_path = []

    vertex_id = vertex.getId()  # Get the vertex ID for comparisons

    # Check if vertex has already been visited
    if vertex_id in visited:
        return False, []  # Stop further exploration if already visited

    visited.add(vertex_id)  # Add the current vertex ID to the visited set
    path.append(vertex_id)  # Append the current vertex's ID to the path
    dfs_path.append(vertex_id)

    # Print current vertex and its heuristic
    # print(f"Exploring {vertex_id} with heuristic: {vertex.getHeuristic()}")
    print(f"Exploring {vertex_id}")

    # Check if the goal is reached
    if vertex_id == goal.getId():  # Fixed goal comparison
        print(" -> ".join(path))
        print(f"Goal {goal.getId()} reached with total distance: {distance}")
        return True, path  # Return True and the path when goal is reached

    # Get all connected vertices and sort them alphabetically
    vertex_connect = list(graph.getVertex(vertex_id).connectedTo.items())
    vertex_connect.sort(key=lambda x: x[0].getId())  # Sort by the vertex ID

    # Recursively explore the neighbors
    for next_vertex, weight in vertex_connect:
        next_vertex_id = next_vertex.getId()  # Get the next vertex ID
        if next_vertex_id not in visited:  # Only explore unvisited neighbors
            # Recur with updated distance
            found, found_path = DFS(next_vertex, graph, visited, path, distance + weight, goal, dfs_path)
            if found:
                return True, found_path  # Stop further recursion and return the path if goal is found

    dfs_path.pop()
    path.pop()  # Remove the vertex if it doesn't lead to the goal
    return False, []  # Return False and an empty list if goal is not found


def main(graph):
    start_name = input("Enter start node: ")
    goal_name = input("Enter goal node: ")

    start = None
    goal = None

    for vertex_id in graph.getVertices():
        vertex = graph.getVertex(vertex_id)
        if vertex.getId() == start_name:
            start = vertex
        if vertex.getId() == goal_name:
            goal = vertex

    if start is None or goal is None:
        print(f"Error: Start vertex '{start_name}' or goal vertex '{goal_name}' does not exist.")
        return

    dfs_path = []

    found, path = DFS(start, graph, goal=goal, dfs_path=dfs_path)
    if not found:
        print(f"Goal {goal.getId()} not reachable from the start location.")
    else:
        return path


