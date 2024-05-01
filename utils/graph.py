import random
import networkx as nx
import matplotlib.pyplot as plt


def generate_random_graph_no_isolated_nodes(num_nodes, num_edges):
    """
    Generates a random graph with the given number of nodes and edges,
    ensuring that no nodes are isolated.

    Args:
        num_nodes (int): Number of nodes in the graph.
        num_edges (int): Number of edges in the graph.

    Returns:
        networkx.Graph: A random graph with the specified number of nodes and edges,
        and random weights on the edges. No nodes are isolated.
    """
    # Create an empty graph
    G = nx.Graph()

    # Add nodes to the graph
    nodes = list(range(num_nodes))
    G.add_nodes_from(nodes)

    # Add edges to the graph
    edges = []
    while len(edges) < num_edges:
        u = random.randint(0, num_nodes - 1)
        v = random.randint(0, num_nodes - 1)
        if u != v and (u, v) not in edges and (v, u) not in edges:
            edges.append((u, v))
            gain = random.uniform(0.1, 1.0)
            cost = random.uniform(0.1, 1.0)
            G.add_edge(u, v, gain = gain, cost = cost)

    # Check for isolated nodes
    isolated_nodes = list(nx.isolates(G))

    # Connect isolated nodes to random neighbors
    while isolated_nodes:
        node = isolated_nodes.pop()
        neighbor = random.choice(list(G.nodes() - {node}))
        gain = random.uniform(0.1, 1.0)
        cost = random.uniform(0.1, 1.0)
        G.add_edge(u, v, gain=gain, cost=cost)

    return G


def visualize_graph_and_path(G, path):
    """
    Visualizes a given graph and highlights a specified path.

    Args:
        G (networkx.Graph): The graph to visualize.
        path (list): A list of nodes representing the path to highlight.

    Returns:
        None
    """
    # Create a copy of the graph to avoid modifying the original
    G_copy = G.copy()

    # Draw the graph
    pos = nx.spring_layout(G_copy)
    nx.draw(G_copy, pos, with_labels=True, node_color='lightgray', edge_color='gray')

    # Highlight the path
    path_edges = list(zip(path, path[1:]))
    nx.draw_networkx_nodes(G_copy, pos, nodelist=path, node_color='r')
    nx.draw_networkx_edges(G_copy, pos, edgelist=path_edges, edge_color='r', width=2)

    # Show the plot
    plt.axis('off')
    plt.show()

def optimize_path(G, start, exit):
    paths = nx.all_simple_paths(G, start, exit)
    for path in paths:
        max_gain_ratio = 0
        cost = 0
        gain = 0
        for i in range(len(path)-1):
            data = G.get_edge_data(path[i], path[i + 1])
            cost += data['cost']
            gain += data['gain']
        e_path = path if (gain / cost) > max_gain_ratio else None
        max_gain_ratio = (gain / cost) if (gain / cost) > max_gain_ratio else max_gain_ratio
    visualize_graph_and_path(G, e_path)
    edge_path = []
    for i in range(len(e_path)-1):
        edge_path.append(G.get_edge_data(e_path[i], e_path[i+1])['entry_waypoint'])
    
    return edge_path
        
        

if __name__ == '__main__':
    random.seed(0)
    nodes_num = 10
    edges_num = 20
    G = generate_random_graph_no_isolated_nodes(nodes_num, edges_num)
    start = 1
    exit = 9
    try:
        e_path = nx.eulerian_path(G)
        edges = [edge for edge in e_path]
        if start != edges[0][0]:
            paths = nx.all_simple_paths(G, start, exit)
            for path in paths:
                max_gain_ratio = 0
                cost = 0
                gain = 0
                for i in range(len(path)-1):
                    data = G.get_edge_data(path[i], path[i+1])
                    cost += data['cost']
                    gain += data['gain']
                e_path = path if (gain / cost) > max_gain_ratio else None
                max_gain_ratio = (gain / cost) if (gain / cost) > max_gain_ratio else max_gain_ratio
        visualize_graph_and_path(G, e_path)
    except:
        paths = nx.all_simple_paths(G, start, exit)
    for path in paths:
        max_gain_ratio = 0
        cost = 0
        gain = 0
        for i in range(len(path)-1):
            data = G.get_edge_data(path[i], path[i + 1])
            cost += data['cost']
            gain += data['gain']
        e_path = path if (gain / cost) > max_gain_ratio else None
        max_gain_ratio = (gain / cost) if (gain / cost) > max_gain_ratio else max_gain_ratio
    visualize_graph_and_path(G, e_path)

    print(f"max_gain_ratio: {max_gain_ratio}")


