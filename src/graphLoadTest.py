import osmnx as ox
import networkx as nx
import math
import matplotlib.pyplot as plt
import myAlgorithms as ma
import graph_utils as gu
import os

# 1. Get the directory of the current script (e.g., /project/src)
current_dir = os.path.dirname(os.path.abspath(__file__))

# 2. Go up one level to the main repo folder
repo_root = os.path.dirname(current_dir)

# 3. Construct path to graph file in another directory (e.g., /project/data)
graph_path = os.path.join(repo_root, 'GraphML_Archive', 'New York City_NY_USA.graphml')


# --- Load and prepare graph ---
G = ox.load_graphml(graph_path)
G = ox.add_edge_speeds(G)
G = ox.add_edge_travel_times(G)
G_projected = ox.project_graph(G) # Projecting graphs converts graph from (lat/lon) to MultiDiGraph

# --- Print basic stats of projected graph ---
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



my_dijkstra_path, my_dijkstra_path_cost = ma.dijkstra(G_projected, origin_node, dest_node, weight='travel_time')

# --- Runs both networkx algorithms ---
networkx_dijksta_path = nx.shortest_path(G_projected, source=origin_node, target=dest_node,
                                  weight='travel_time')
networkx_astar_path    = nx.astar_path(G_projected, source=origin_node, target=dest_node,
                               heuristic=euclidean_heuristic, weight='travel_time')


dijkstra_time = gu.path_travel_time(G_projected, networkx_dijksta_path)
astar_time    = gu.path_travel_time(G_projected, networkx_astar_path)
my_dijkstra_path_time  = gu.path_travel_time(G_projected, my_dijkstra_path)


# Prints some basic data about the paths created
print(f"Dijkstra — {len(networkx_dijksta_path)} nodes, {dijkstra_time/60:.2f} min")
print(f"A*       — {len(networkx_astar_path)} nodes, {astar_time/60:.2f} min")
print(f"Nodes in my path : {len(my_dijkstra_path)}")
print(f"My Path Travel time   : {my_dijkstra_path_cost:.1f} seconds ({my_dijkstra_path_cost/60:.2f} min)")


# --- Get bounding box directly from the route plot axis limits ---
fig, axes = plt.subplots(1, 3, figsize=(16, 8), facecolor='black')

# Plot your custom Dijkstra result
ox.plot_graph_route(
    G_projected, my_dijkstra_path,
    route_color='lime',
    route_linewidth=3,
    node_size=0,
    bgcolor='black',
    ax=axes[0],
    show=False,
    close=False
)
axes[0].set_title(f"My Dijkstra ({my_dijkstra_path_cost/60:.2f} min)", color='white', fontsize=13)

# Plot Dijkstra first to get natural zoom limits
ox.plot_graph_route(
    G_projected, networkx_dijksta_path,
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
    G_projected, networkx_astar_path,
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
# #ox.plot_graph(
#     G_cropped,
#     ax=axes[0],
#     node_size=0,
#     edge_color='black',
#     edge_linewidth=0.5,
#     bgcolor='black',
#     show=False,
#     close=False
# )
# axes[0].set_title("Base Street Map", color='white', fontsize=13)

plt.suptitle("Times Square → Brooklyn Bridge", color='white', fontsize=15)
plt.tight_layout()
plt.savefig('comparison_with_personal_dijkstra.png', dpi=150, bbox_inches='tight')
plt.show()