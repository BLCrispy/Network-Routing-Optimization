import osmnx as ox
import networkx as nx
import math
import matplotlib.pyplot as plt
import myAlgorithms as ma
import time
import os

# 1. Get the directory of the current script (e.g., /project/src)
current_dir = os.path.dirname(os.path.abspath(__file__))

# 2. Go up one level to the main repo folder
repo_root = os.path.dirname(current_dir)

# 3. Construct path to graph file in another directory (e.g., /project/data)
graph_path = os.path.join(repo_root, 'GraphML_Archive', 'New York City_NY_USA.graphml')

# -----------------------------------------------------------------------
# Load and project graph
# -----------------------------------------------------------------------
G = ox.load_graphml(graph_path)
G = ox.add_edge_speeds(G)
G = ox.add_edge_travel_times(G)
print('Projecting Graph...')
G_projected = ox.project_graph(G)
print('Graph Projected!\n')

origin_node = ox.nearest_nodes(G, X=-73.9855, Y=40.7580)  # Times Square
dest_node   = ox.nearest_nodes(G, X=-73.9969, Y=40.7061)  # Brooklyn Bridge

# -----------------------------------------------------------------------
# Run all four algorithms
# -----------------------------------------------------------------------

# --- Dijkstra's ---
print("=" * 50)
print("Running Dijkstra's...")
t0 = time.time()
dijkstra_path, dijkstra_cost = ma.dijkstra(
    G_projected, origin_node, dest_node, weight='travel_time'
)
dijkstra_time = time.time() - t0
print(f"Dijkstra's  : {dijkstra_cost/60:.2f} min | "
      f"{len(dijkstra_path)} nodes | {dijkstra_time:.4f}s runtime")

# --- A* ---
print("\nRunning A*...")
t0 = time.time()
astar_path, astar_cost = ma.astar(
    G_projected, origin_node, dest_node, weight='travel_time'
)
astar_time = time.time() - t0
print(f"A*          : {astar_cost/60:.2f} min | "
      f"{len(astar_path)} nodes | {astar_time:.4f}s runtime")

# --- Bellman-Ford ---
print("\nRunning Bellman-Ford...")
t0 = time.time()
bf_path, bf_cost, has_neg_cycle = ma.bellman_ford(
    G_projected, origin_node, dest_node, weight='travel_time'
)
bf_time = time.time() - t0
print(f"Bellman-Ford: {bf_cost/60:.2f} min | "
      f"{len(bf_path)} nodes | {bf_time:.4f}s runtime")

# --- ACO ---
print("\nRunning ACO diagnostics and optimization...")
params = ma.diagnose_aco_parameters(
    G_projected, origin_node, dest_node,
    weight='travel_time', padding=3000
)
t0 = time.time()
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
    max_steps        = params['max_steps'],
)
aco_time = time.time() - t0

# -----------------------------------------------------------------------
# Results summary
# -----------------------------------------------------------------------
print("\n" + "=" * 50)
print("RESULTS SUMMARY")
print("=" * 50)
print(f"{'Algorithm':<14} {'Cost':>10} {'Nodes':>8} {'Runtime':>10} {'Gap':>10}")
print("-" * 50)

results = [
    ("Dijkstra's",   dijkstra_cost, dijkstra_path, dijkstra_time),
    ("A*",           astar_cost,    astar_path,    astar_time),
    ("Bellman-Ford", bf_cost,       bf_path,       bf_time),
]
if aco_cost < math.inf:
    results.append(("ACO (hybrid)", aco_cost, aco_path, aco_time))

optimal = dijkstra_cost
for name, cost, path, runtime in results:
    gap = (cost - optimal) / optimal * 100 if optimal > 0 else 0
    gap_str = f"{gap:+.1f}%" if name != "Dijkstra's" else "baseline"
    print(f"{name:<14} {cost/60:>9.2f}m {len(path):>8} "
          f"{runtime:>9.4f}s {gap_str:>10}")

# -----------------------------------------------------------------------
# Verification — all exact algorithms should agree
# -----------------------------------------------------------------------
print("\n--- Verification ---")
print(f"Dijkstra's == A*          : "
      f"{math.isclose(dijkstra_cost, astar_cost, rel_tol=1e-6)}")
print(f"Dijkstra's == Bellman-Ford: "
      f"{math.isclose(dijkstra_cost, bf_cost, rel_tol=1e-6)}")
print(f"Negative cycles detected  : {has_neg_cycle}")

# -----------------------------------------------------------------------
# ACO convergence plot
# -----------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(10, 4), facecolor='black')
ax.set_facecolor('black')

valid_history = [(i, c/60) for i, c in enumerate(history) if c is not None]
if valid_history:
    iters, costs = zip(*valid_history)
    ax.plot(iters, costs, color='lime', linewidth=2, label='ACO best')

ax.axhline(dijkstra_cost/60, color='cyan',   linewidth=1.5,
           linestyle='--', label=f"Dijkstra's ({dijkstra_cost/60:.2f} min)")
ax.axhline(astar_cost/60,    color='orange', linewidth=1.5,
           linestyle=':',  label=f"A* ({astar_cost/60:.2f} min)")

ax.set_xlabel('Iteration', color='white')
ax.set_ylabel('Travel Time (min)', color='white')
ax.set_title('ACO Convergence vs Exact Algorithms', color='white', fontsize=13)
ax.tick_params(colors='white')
ax.legend(facecolor='black', labelcolor='white')
plt.tight_layout()
plt.savefig('convergence.png', dpi=150, bbox_inches='tight')
plt.show()

# -----------------------------------------------------------------------
# Route comparison — all four algorithms
# -----------------------------------------------------------------------
all_results = [
    (dijkstra_path, 'cyan',   f"Dijkstra's\n{dijkstra_cost/60:.2f} min | "
                               f"{dijkstra_time:.4f}s"),
    (astar_path,    'orange', f"A*\n{astar_cost/60:.2f} min | "
                               f"{astar_time:.4f}s"),
    (bf_path,       'magenta',f"Bellman-Ford\n{bf_cost/60:.2f} min | "
                               f"{bf_time:.4f}s"),
]
if aco_path:
    all_results.append(
        (aco_path, 'lime', f"ACO (hybrid)\n{aco_cost/60:.2f} min | "
                            f"{aco_time:.1f}s")
    )

n_plots = len(all_results)
fig, axes = plt.subplots(1, n_plots,
                          figsize=(8 * n_plots, 8),
                          facecolor='black')
if n_plots == 1:
    axes = [axes]

# Get zoom window from Dijkstra plot to match all panels
ox.plot_graph_route(
    G_projected, dijkstra_path,
    route_color='cyan', route_linewidth=3,
    node_size=0, bgcolor='black',
    ax=axes[0], show=False, close=False
)
axes[0].set_title(all_results[0][2], color='white', fontsize=11)
xlim = axes[0].get_xlim()
ylim = axes[0].get_ylim()

for i, (path, color, label) in enumerate(all_results[1:], start=1):
    ox.plot_graph_route(
        G_projected, path,
        route_color=color, route_linewidth=3,
        node_size=0, bgcolor='black',
        ax=axes[i], show=False, close=False
    )
    axes[i].set_title(label, color='white', fontsize=11)
    axes[i].set_xlim(xlim)
    axes[i].set_ylim(ylim)

plt.suptitle("Times Square → Brooklyn Bridge — Algorithm Comparison",
             color='white', fontsize=14)
plt.tight_layout()
plt.savefig('algorithm_comparison.png', dpi=150, bbox_inches='tight')
plt.show()