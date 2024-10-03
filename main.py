from pythonds.graphs import Graph
from pythonds.basic import Stack, Queue

# undirected_connect Connects two nodes together (Undirected)
# graph is the graph
# one is first node
# two is second node
def undirected_connect(graph, one, two):
    graph.addEdge(one, two)
    graph.addEdge(two, one)

# directed_weighted_connect Connects a node to another node (Directed from one to two)
# graph is the graph
# one is first node
# two is second node
# weight is the weight of the edge
def directed_weighted_connect(graph, one, two, weight):
    graph.addEdge(one, two, weight)

# Main function
def main():
    graph = Graph()

    undirected_connect(graph, "Name1", "Name2")
    undirected_connect(graph, "Name1", "Name3")
    undirected_connect(graph, "Name2", "Name3")

if __name__ == '__main__':
    main()