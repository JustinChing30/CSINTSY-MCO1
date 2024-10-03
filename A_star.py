from pythonds.graphs import Graph, Vertex

# extends the Vertex class imported from pythonds to include a heuristic value
class VertexHeuristic(Vertex):
    def _init__(self, node_name, heuristic_val):
        super().__init__(node_name)
        self.heuristic_val = heuristic_val

# undirected_connect Connects two nodes together (Undirected)
# graph is the graph
# one is first node
# two is second node
def undirected_connect(graph, one, two):
    graph.addEdge(one, two)
    graph.addEdge(two, one)

# Main function
def main():
    graph = Graph()

if __name__ == '__main__':
    main()
