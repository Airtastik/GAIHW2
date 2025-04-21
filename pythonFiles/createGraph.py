import random
import networkx as nx
import matplotlib.pyplot as plt

def generate_random_digraph(N, edge_probability=0.3, weight_range=(1, 10)):
    """
    Generates a random weighted directed graph with N vertices.
    
    :param N: Number of vertices
    :param edge_probability: Probability of an edge existing between two nodes
    :param weight_range: Tuple representing the range of edge weights (min, max)
    :return: A NetworkX directed graph
    """
    G = nx.DiGraph()
    G.add_nodes_from(range(N))
    
    for i in range(N):
        for j in range(N):
            if i != j and random.random() < edge_probability:  # Avoid self-loops
                weight = random.randint(*weight_range)
                G.add_edge(i, j, weight=weight)
    
    return G

def draw_graph(G):
    """Draws the directed weighted graph."""
    pos = nx.spring_layout(G)
    edge_labels = {(u, v): G[u][v]['weight'] for u, v in G.edges}
    
    nx.draw(G, pos, with_labels=True, node_color='lightblue', node_size=700, edge_color='gray', arrowsize=20)
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
    plt.show()

if __name__ == "__main__":
    N = int(input("Enter the number of vertices: "))
    edge_probability = float(input("Enter the probability of edge creation (0-1): "))
    G = generate_random_digraph(N, edge_probability)
    draw_graph(G)
