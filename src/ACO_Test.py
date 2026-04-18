import osmnx as ox
import networkx as nx
import math
import matplotlib.pyplot as plt
import myAlgorithms as ma



G = ox.load_graphml('New York City_NY_USA.graphml')
G = ox.add_edge_speeds(G)
G = ox.add_edge_travel_times(G)
print('Projecting Graph...')
G_projected = ox.project_graph(G)
print('Graph Projected!')

origin_node = ox.nearest_nodes(G, X=-73.9855, Y=40.7580)  # Times Square
dest_node   = ox.nearest_nodes(G, X=-73.9969, Y=40.7061)  # Brooklyn Bridge

# Quick diagnostic — run before ACO
test_path = nx.shortest_path(G_projected, origin_node, dest_node, weight='travel_time')
print(f"Dijkstra path length: {len(test_path)} nodes")

# Check how many nodes in the Dijkstra path appear more than once
duplicates = len(test_path) - len(set(test_path))
print(f"Repeated nodes in optimal path: {duplicates}")

# Check what percentage of the subgraph is reachable from origin
G_sub = ma.build_aco_subgraph(G_projected, origin_node, dest_node, padding=3000)
reachable = nx.single_source_shortest_path_length(G_sub, origin_node, cutoff=50)
print(f"Nodes reachable from origin within 50 hops: {len(reachable)}/{G_sub.number_of_nodes()}")
print(f"Destination reachable: {dest_node in reachable}")


# Run diagnostics and get recommended parameters
params = ma.diagnose_aco_parameters(
    G_projected, origin_node, dest_node,
    weight='travel_time', padding=3000
)


# Feed recommended parameters directly into ACO
aco_path, aco_cost, history = ma.ant_colony_optimization(
    G_projected,
    origin_node,
    dest_node,
    n_ants           = params['n_ants'],
    n_iterations     = params['n_iterations'],
    alpha            = params['alpha'],
    beta             = params['beta'],
    evaporation_rate = params['evaporation_rate'],
    deposit_weight   = params['deposit_weight'],
    weight           = 'travel_time',
    padding          = 3000,
    max_steps        = params['max_steps']
)

dijkstra_path = nx.shortest_path(G_projected, origin_node, dest_node, weight='travel_time')
dijkstra_cost = nx.shortest_path_length(G_projected, origin_node, dest_node, weight='travel_time')

print(f"\nDijkstra : {dijkstra_cost/60:.2f} min")
print(f"ACO      : {aco_cost/60:.2f} min" if aco_cost < math.inf else "ACO      : no path found")

# --- Gives efficiency of ACO compared to Dijkstra ---
if aco_cost < math.inf:
    gap = (aco_cost - dijkstra_cost) / dijkstra_cost * 100
    print(f"Gap      : {gap:.1f}% above optimal")


# --- Convergence plot ---
fig, ax = plt.subplots(figsize=(10, 4), facecolor='black')
ax.set_facecolor('black')

# Filter out None values for plotting
valid_history = [(i, c/60) for i, c in enumerate(history) if c is not None]
if valid_history:
    iters, costs = zip(*valid_history)
    ax.plot(iters, costs, color='lime', linewidth=2, label='ACO best')
    ax.axhline(dijkstra_cost/60, color='cyan', linewidth=1.5,
               linestyle='--', label=f'Dijkstra ({dijkstra_cost/60:.2f} min)')

ax.set_xlabel('Iteration', color='white')
ax.set_ylabel('Travel Time (min)', color='white')
ax.set_title('ACO Convergence', color='white', fontsize=13)
ax.tick_params(colors='white')
ax.legend(facecolor='black', labelcolor='white')
plt.tight_layout()
plt.savefig('aco_convergence.png', dpi=150, bbox_inches='tight')
plt.show()

# --- Route comparison (only plot ACO if it found a path) ---
n_plots = 2 if aco_path else 1
fig, axes = plt.subplots(1, n_plots, figsize=(8 * n_plots, 8), facecolor='black')

if n_plots == 1:
    axes = [axes]

ox.plot_graph_route(
    G_projected, dijkstra_path,
    route_color='cyan', route_linewidth=3,
    node_size=0, bgcolor='black',
    ax=axes[0], show=False, close=False
)
axes[0].set_title(f"Dijkstra ({dijkstra_cost/60:.2f} min)", color='white', fontsize=13)

if aco_path:
    ox.plot_graph_route(
        G_projected, aco_path,
        route_color='lime', route_linewidth=3,
        node_size=0, bgcolor='black',
        ax=axes[1], show=False, close=False
    )
    axes[1].set_title(f"ACO ({aco_cost/60:.2f} min)", color='white', fontsize=13)

plt.suptitle("Dijkstra vs ACO — Times Square → Brooklyn Bridge",
             color='white', fontsize=15)
plt.tight_layout()
plt.savefig('aco_vs_dijkstra.png', dpi=150, bbox_inches='tight')
plt.show()