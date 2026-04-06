import osmnx as ox
import networkx as nx
import heapq
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


def dijkstra(G, origin, destination, weight='travel_time'):
    """
    Dijkstra's shortest path algorithm implemented from scratch.
    
    Parameters:
        G           : projected OSMnx MultiDiGraph
        origin      : origin node ID
        destination : destination node ID
        weight      : edge attribute to minimize ('travel_time', 'length', etc.)
    
    Returns:
        path        : list of node IDs representing the shortest path
        cost        : total cost of the path (in units of the weight attribute)
    """

    # --- Step 1: Initialize data structures ---

    # Priority queue: stores (cost, node)
    # heapq in Python is a min-heap so the lowest cost is always popped first
    pq = []
    heapq.heappush(pq, (0, origin))

    # Cost to reach each node (start with infinity for all)
    costs = {node: math.inf for node in G.nodes()}
    costs[origin] = 0

    # Track which node we came from to reconstruct the path later
    previous = {node: None for node in G.nodes()}

    # Track fully visited/settled nodes
    visited = set()

    # --- Step 2: Main loop ---
    while pq:

        # Pop the node with the lowest cost so far
        current_cost, current_node = heapq.heappop(pq)

        # Skip if we already settled this node
        # (stale entries can exist in the heap from earlier updates)
        if current_node in visited:
            continue

        # If we reached the destination we are done
        if current_node == destination:
            break

        # Mark as settled
        visited.add(current_node)

        # --- Step 3: Relax neighbors ---
        # G[current_node] returns a dict of {neighbor: {edge_key: edge_data}}
        # since OSMnx uses a MultiDiGraph there can be multiple edges
        # between the same two nodes so we take the cheapest one
        for neighbor, edges in G[current_node].items():

            if neighbor in visited:
                continue

            # Find the minimum weight edge between current_node and neighbor
            edge_cost = min(
                edge_data.get(weight, math.inf)
                for edge_data in edges.values()
            )

            # Calculate new cost to reach this neighbor via current_node
            new_cost = current_cost + edge_cost

            # If this is cheaper than what we knew before, update it
            if new_cost < costs[neighbor]:
                costs[neighbor] = new_cost
                previous[neighbor] = current_node
                heapq.heappush(pq, (new_cost, neighbor))

    # --- Step 4: Reconstruct path by walking backwards from destination ---
    path = []
    node = destination

    # If destination was never reached, return empty
    if costs[destination] == math.inf:
        print(f"No path found from {origin} to {destination}")
        return [], math.inf

    while node is not None:
        path.append(node)
        node = previous[node]

    path.reverse()  # We built it backwards so flip it

    return path, costs[destination]


my_path, my_cost = dijkstra(G_projected, origin_node, dest_node, weight='travel_time')

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
my_path_time  = path_travel_time(G_projected, my_path)

print(f"Dijkstra — {len(path_dijkstra)} nodes, {dijkstra_time/60:.2f} min")
print(f"A*       — {len(path_astar)} nodes, {astar_time/60:.2f} min")
print(f"Nodes in my path : {len(my_path)}")
print(f"My Path Travel time   : {my_cost:.1f} seconds ({my_cost/60:.2f} min)")


# --- Get bounding box directly from the route plot axis limits ---
fig, axes = plt.subplots(1, 3, figsize=(16, 8), facecolor='black')

# Plot your custom Dijkstra result
ox.plot_graph_route(
    G_projected, my_path,
    route_color='lime',
    route_linewidth=3,
    node_size=0,
    bgcolor='black',
    ax=axes[0],
    show=False,
    close=False
)
axes[0].set_title(f"My Dijkstra ({my_cost/60:.2f} min)", color='white', fontsize=13)

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