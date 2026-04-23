import osmnx as ox
import networkx as nx

def get_graph(place_name):
    """Download a street network graph for a place."""
    G = ox.graph_from_place(place_name, network_type="drive")
    return G

# --- Calculate total travel time ---
def path_travel_time(G, path):
    total = 0
    for u, v in zip(path[:-1], path[1:]):
        edge_data = min(G[u][v].values(), key=lambda e: e.get('travel_time', float('inf')))
        total += edge_data.get('travel_time', 0)
    return total

# --- Calculate total length of path --- 
def path_length(G, path):
    total_length = sum(G[u][v][0]['length'] for u, v in zip(path[:-1], path[1:]))
    return total_length