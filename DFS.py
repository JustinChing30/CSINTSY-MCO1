from pythonds.graphs import Graph, Vertex

# undirected_connect Connects two nodes together (Undirected)
# graph is the graph
# one is first node abbreviation
# two is second node abbreviation
def undirected_connect(graph, one, two, weight=1):
    graph.addEdge(one, two, weight)
    graph.addEdge(two, one, weight)

# DFS function with distance tracking and goal checking
def DFS(vertex, visited=None, path = None, distance=0, goal=None):
    if visited is None:
        visited = set()  # Initialize visited set if not provided
        
    if path is None:
        path = []

    visited.add(vertex)
    path.append(vertex.getId())  # Add current vertex to path

    # Check if the goal is reached
    if vertex.getId() == goal:
        print(" -> ".join(path))  # Print the path
        print(f"Goal {goal} reached with total distance: {distance}")
        return True  # Stop the search once the goal is reached

    # Get all connected vertices and sort by the edge weight
    vertex_connect = sorted(vertex.connectedTo.items(), key=lambda x: x[0].getId())

    for next_vertex, weight in vertex_connect:
        if next_vertex not in visited:
            # Recur with updated distance
            if DFS(next_vertex, visited, path, distance + weight, goal):
                return True  # Stop further recursion once the goal is reached
    path.pop()
    return False  # Return False if goal not found in this path

# Main function
def main():
    # Create graph
    graph = Graph()

    # Create a dictionary to map abbreviations to full names
    heuristic_values = {
        "Arad": 366, "Zerind": 374, "Oradea": 380, "Sibiu": 253, "Timisoara": 329, "Lugoj": 244,
        "Mehadia": 241, "Dobreta": 242, "Craiova": 160, "Rimnicu Vilcea": 193, "Pitesti": 100,
        "Fagaras": 176, "Bucharest": 0, "Giurgiu": 77, "Urziceni": 80, "Vaslui": 199, "Iasi": 226,
        "Neamt": 234, "Hirsova": 151, "Eforie": 161
    }   
    
    for location in heuristic_values:
        graph.addVertex(location, heuristic_values[location])

    # Add vertices to the graph using abbreviations

    # Add undirected edges with abbreviations and weights
    undirected_connect(graph, "Arad", "Zerind", 75)
    undirected_connect(graph, "Arad", "Sibiu", 140)
    undirected_connect(graph, "Arad", "Timisoara", 118)
    undirected_connect(graph, "Zerind", "Oradea", 71)
    undirected_connect(graph, "Oradea", "Sibiu", 151)
    undirected_connect(graph, "Timisoara", "Lugoj", 111)
    undirected_connect(graph, "Lugoj", "Mehadia", 70)
    undirected_connect(graph, "Mehadia", "Dobreta", 75)
    undirected_connect(graph, "Dobreta", "Craiova", 120)
    undirected_connect(graph, "Craiova", "Rimnicu Vilcea", 146)
    undirected_connect(graph, "Craiova", "Pitesti", 138)
    undirected_connect(graph, "Rimnicu Vilcea", "Sibiu", 80)
    undirected_connect(graph, "Rimnicu Vilcea", "Pitesti", 97)
    undirected_connect(graph, "Pitesti", "Bucharest", 101)
    undirected_connect(graph, "Fagaras", "Sibiu", 99)
    undirected_connect(graph, "Fagaras", "Bucharest", 211)
    undirected_connect(graph, "Bucharest", "Giurgiu", 90)
    undirected_connect(graph, "Bucharest", "Urziceni", 85)
    undirected_connect(graph, "Urziceni", "Hirsova", 98)
    undirected_connect(graph, "Hirsova", "Eforie", 86)
    undirected_connect(graph, "Urziceni", "Vaslui", 142)
    undirected_connect(graph, "Vaslui", "Iasi", 92)
    undirected_connect(graph, "Iasi", "Neamt", 87)
    # (Add more edges as needed)

    # Start DFS from "Arad" and end when reaches "Bucharest"
    start_vertex = graph.getVertex("Arad")
    goal = "Bucharest"
    
    if not DFS(start_vertex, goal=goal):
        print(f"Goal {goal} not reachable from the start location.")

if __name__ == '__main__':
    main()
