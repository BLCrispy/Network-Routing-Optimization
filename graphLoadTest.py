import osmnx as ox
import networkx as nx
import math
import matplotlib.pyplot as plt

# --- Load and prepare graph ---
G = ox.load_graphml('New York City_NY_USA.graphml')
G = ox.add_edge_speeds(G)
G = ox.add_edge_travel_times(G)
G_projected = ox.project_graph(G)

print(ox.basic_stats(G_projected))

# --- Find nodes using UNPROJECTED graph G (lat/lon) ---
origin_node = ox.nearest_nodes(G, X=-73.9855, Y=40.7580)   # Times Square
dest_node   = ox.nearest_nodes(G, X=-73.9969, Y=40.7061)   # Brooklyn Bridge

print(f"Origin node (Times Square):         {origin_node}")
print(f"Destination node (Brooklyn Bridge): {dest_node}")

# --- Heuristic for A* (uses projected coords for accurate meter distances) ---
def euclidean_heuristic(u, v):
    x1, y1 = G_projected.nodes[u]['x'], G_projected.nodes[u]['y']
    x2, y2 = G_projected.nodes[v]['x'], G_projected.nodes[v]['y']
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

# --- Run both algorithms ---
path_dijkstra = nx.shortest_path(G_projected, source=origin_node, target=dest_node,
                                  weight='travel_time')
path_astar    = nx.astar_path(G_projected, source=origin_node, target=dest_node,
                               heuristic=euclidean_heuristic, weight='travel_time')

# --- Calculate total travel time ---
def path_travel_time(G, path):
    total = 0
    for u, v in zip(path[:-1], path[1:]):
        edge_data = min(G[u][v].values(), key=lambda e: e.get('travel_time', float('inf')))
        total += edge_data.get('travel_time', 0)
    return total

dijkstra_time = path_travel_time(G_projected, path_dijkstra)
astar_time    = path_travel_time(G_projected, path_astar)

print(f"Dijkstra — {len(path_dijkstra)} nodes, {dijkstra_time/60:.2f} min")
print(f"A*       — {len(path_astar)} nodes, {astar_time/60:.2f} min")


# --- Get bounding box directly from the route plot axis limits ---
fig, axes = plt.subplots(1, 3, figsize=(24, 8), facecolor='black')

# Plot Dijkstra first to get natural zoom limits
ox.plot_graph_route(
    G_projected, path_dijkstra,
    route_color='cyan',
    route_linewidth=3,
    node_size=0,
    bgcolor='black',
    ax=axes[1],
    show=False,
    close=False
)
axes[1].set_title(f"Dijkstra ({dijkstra_time/60:.2f} min)", color='white', fontsize=13)

# Steal the zoom
xlim = axes[1].get_xlim()
ylim = axes[1].get_ylim()

# Plot A*
ox.plot_graph_route(
    G_projected, path_astar,
    route_color='orange',
    route_linewidth=3,
    node_size=0,
    bgcolor='black',
    ax=axes[2],
    show=False,
    close=False
)
axes[2].set_title(f"A* ({astar_time/60:.2f} min)", color='white', fontsize=13)

# --- Crop the graph to just the route area before plotting ---
# Convert axis limits back to a bbox and extract only nodes in that window
nodes_in_view = {
    node: data for node, data in G_projected.nodes(data=True)
    if xlim[0] <= data['x'] <= xlim[1] and ylim[0] <= data['y'] <= ylim[1]
}
G_cropped = G_projected.subgraph(nodes_in_view.keys())

# Plot the cropped subgraph — much faster and fills the view correctly
ox.plot_graph(
    G_cropped,
    ax=axes[0],
    node_size=0,
    edge_color='black',
    edge_linewidth=0.5,
    bgcolor='black',
    show=False,
    close=False
)
axes[0].set_title("Base Street Map", color='white', fontsize=13)

plt.suptitle("Times Square → Brooklyn Bridge", color='white', fontsize=15)
plt.tight_layout()
plt.savefig('comparison_with_base.png', dpi=150, bbox_inches='tight')
plt.show()