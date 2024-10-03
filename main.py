from pythonds.graphs import Graph
from pythonds.basic import Stack, Queue

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

    undirected_connect(graph, "Eatery1", "Name2")

if __name__ == '__main__':
    main()