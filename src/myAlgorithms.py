import osmnx as ox
import networkx as nx
import heapq
import math

import random
import time
import statistics
import matplotlib.pyplot as plt


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

# Astar Algorithm Function
def astar(G, origin, destination, weight='travel_time'):
    """
    A* shortest path algorithm implemented from scratch.
    Uses Euclidean distance between projected coordinates as the heuristic.
    Requires a projected graph (meters) for accurate heuristic values.

    Parameters:
        G           : projected OSMnx MultiDiGraph
        origin      : origin node ID
        destination : destination node ID
        weight      : edge attribute to minimize ('travel_time', 'length', etc.)

    Returns:
        path        : list of node IDs representing the shortest path
        cost        : total cost of the path in units of the weight attribute
    """

    # --- Heuristic: Euclidean distance scaled to match weight units ---
    # For travel_time weight: convert meter distance to seconds using
    # an assumed average speed. 50 kph = 13.89 m/s is a reasonable
    # urban average that keeps the heuristic admissible (never overestimates)
    # For length weight: heuristic is just straight line distance in meters
    AVG_SPEED_MPS = 50 * 1000 / 3600  # 50 kph in meters per second

    dest_x = G.nodes[destination]['x']
    dest_y = G.nodes[destination]['y']

    def heuristic(node):
        nx_ = G.nodes[node]['x']
        ny_ = G.nodes[node]['y']
        euclidean = math.sqrt((nx_ - dest_x) ** 2 + (ny_ - dest_y) ** 2)
        if weight == 'travel_time':
            return euclidean / AVG_SPEED_MPS  # meters / (m/s) = seconds
        return euclidean  # for 'length' weight, meters is correct directly

    # --- Initialize data structures ---
    # Priority queue stores (f_score, node)
    # f_score = g_score + heuristic — A*'s key insight over Dijkstra's
    pq = []
    heapq.heappush(pq, (heuristic(origin), origin))

    # g_score: actual cost from origin to each node (like Dijkstra's costs dict)
    g_scores          = {node: math.inf for node in G.nodes()}
    g_scores[origin]  = 0

    # f_score: g_score + heuristic estimate to destination
    f_scores          = {node: math.inf for node in G.nodes()}
    f_scores[origin]  = heuristic(origin)

    # Track path for reconstruction
    previous = {node: None for node in G.nodes()}

    # Settled nodes — same role as in Dijkstra's
    visited = set()

    # --- Main loop ---
    while pq:
        current_f, current_node = heapq.heappop(pq)

        # Skip stale heap entries
        if current_node in visited:
            continue

        # Reached destination
        if current_node == destination:
            break

        visited.add(current_node)

        # --- Relax neighbors ---
        for neighbor, edges in G[current_node].items():
            if neighbor in visited:
                continue

            # Take cheapest parallel edge (same as Dijkstra's)
            edge_cost = min(
                data.get(weight, math.inf)
                for data in edges.values()
            )

            tentative_g = g_scores[current_node] + edge_cost

            if tentative_g < g_scores[neighbor]:
                g_scores[neighbor]  = tentative_g
                f_scores[neighbor]  = tentative_g + heuristic(neighbor)
                previous[neighbor]  = current_node
                heapq.heappush(pq, (f_scores[neighbor], neighbor))

    # --- Reconstruct path ---
    if g_scores[destination] == math.inf:
        print(f"No path found from {origin} to {destination}")
        return [], math.inf

    path = []
    node = destination
    while node is not None:
        path.append(node)
        node = previous[node]
    path.reverse()

    return path, g_scores[destination]


# Bellman-Ford Algorithm Function
def bellman_ford(G, origin, destination, weight='travel_time'):
    """
    Bellman-Ford shortest path algorithm implemented from scratch.
    Slower than Dijkstra's and A* but handles negative edge weights
    and detects negative cycles — useful as a correctness benchmark.

    On OSM graphs all edge weights are positive so it will always
    agree with Dijkstra's. Its value here is as a verification tool
    and for understanding algorithm tradeoffs.

    Time complexity: O(n * m) where n=nodes, m=edges
    For NYC subgraph: 4146 * 8623 = ~35 million operations per run
    Expect it to be significantly slower than Dijkstra's and A*.

    Parameters:
        G           : projected OSMnx MultiDiGraph
        origin      : origin node ID
        destination : destination node ID
        weight      : edge attribute to minimize ('travel_time', 'length', etc.)

    Returns:
        path        : list of node IDs representing the shortest path
        cost        : total cost of the path in units of the weight attribute
        has_negative_cycle : bool — True if a negative cycle was detected
    """

    nodes = list(G.nodes())
    n     = len(nodes)

    # --- Initialize ---
    costs            = {node: math.inf for node in nodes}
    costs[origin]    = 0
    previous         = {node: None for node in nodes}

    # Precompute edge list once — Bellman-Ford iterates over ALL edges
    # n-1 times so this avoids repeated graph traversal
    edges = []
    for u, v, data in G.edges(data=True):
        edge_cost = data.get(weight, math.inf)
        if edge_cost < math.inf:
            edges.append((u, v, edge_cost))

    # For MultiDiGraph keep only minimum cost edge between each node pair
    # to avoid redundant relaxations
    min_edges = {}
    for u, v, cost in edges:
        if (u, v) not in min_edges or cost < min_edges[(u, v)]:
            min_edges[(u, v)] = cost
    edge_list = [(u, v, c) for (u, v), c in min_edges.items()]

    print(f"Bellman-Ford: {n} nodes, {len(edge_list)} edges")
    print(f"Running {n-1} relaxation passes...")

    # --- Main loop: relax all edges n-1 times ---
    # After k iterations, all shortest paths using at most k edges are optimal.
    # After n-1 iterations, all shortest paths are optimal (assuming no
    # negative cycles — a path can use at most n-1 edges without looping)
    start = time.time()

    for i in range(n - 1):
        # Early termination — if no update happened this pass,
        # all shortest paths are already optimal
        updated = False

        for u, v, edge_cost in edge_list:
            if costs[u] == math.inf:
                continue  # can't relax from unreached node

            new_cost = costs[u] + edge_cost
            if new_cost < costs[v]:
                costs[v]    = new_cost
                previous[v] = u
                updated     = True

        if not updated:
            print(f"  Converged early at iteration {i+1}/{n-1} "
                  f"({time.time()-start:.1f}s)")
            break

        # Progress update every 500 iterations for large graphs
        if (i + 1) % 500 == 0:
            elapsed = time.time() - start
            print(f"  Pass {i+1}/{n-1} | Elapsed: {elapsed:.1f}s")

    # --- Negative cycle detection ---
    # If we can still relax any edge after n-1 passes, a negative cycle exists
    has_negative_cycle = False
    for u, v, edge_cost in edge_list:
        if costs[u] != math.inf and costs[u] + edge_cost < costs[v]:
            has_negative_cycle = True
            print("WARNING: Negative cycle detected — costs are unreliable")
            break

    if not has_negative_cycle:
        print("No negative cycles detected — graph is valid")

    # --- Reconstruct path ---
    if costs[destination] == math.inf:
        print(f"No path found from {origin} to {destination}")
        return [], math.inf, has_negative_cycle

    path = []
    node = destination
    # Guard against infinite loop if negative cycle corrupted previous dict
    visited_reconstruction = set()
    while node is not None:
        if node in visited_reconstruction:
            print("WARNING: Cycle detected during path reconstruction")
            break
        visited_reconstruction.add(node)
        path.append(node)
        node = previous[node]
    path.reverse()

    return path, costs[destination], has_negative_cycle


# EVERYTHING below is for ACO
def build_aco_subgraph(G, origin, destination, padding=3000):
    """
    Builds a subgraph that guarantees no dead-end sinks by:
    1. Cropping to bounding box
    2. Extracting the largest strongly connected component
    3. Verifying origin and destination survive the SCC extraction
    4. Expanding padding if they don't
    """
    for attempt, pad in enumerate([padding, padding * 2, padding * 4, padding * 8]):
        origin_x = G.nodes[origin]['x']
        origin_y = G.nodes[origin]['y']
        dest_x   = G.nodes[destination]['x']
        dest_y   = G.nodes[destination]['y']

        north = max(origin_y, dest_y) + pad
        south = min(origin_y, dest_y) - pad
        east  = max(origin_x, dest_x) + pad
        west  = min(origin_x, dest_x) - pad

        nodes_in_box = [
            node for node, data in G.nodes(data=True)
            if west <= data['x'] <= east and south <= data['y'] <= north
        ]
        G_sub = G.subgraph(nodes_in_box).copy()

        if origin not in G_sub.nodes or destination not in G_sub.nodes:
            print(f"Attempt {attempt+1}: Origin or destination missing, "
                  f"expanding padding to {pad*2}m...")
            continue

        # --- Key fix: extract strongly connected component ---
        # A strongly connected component (SCC) guarantees every node
        # can reach every other node — no dead-end sinks possible.
        # This eliminates the "STUCK — no successors" failure entirely.
        print(f"Attempt {attempt+1}: Extracting strongly connected component...")
        sccs = list(nx.strongly_connected_components(G_sub))

        # Find which SCC contains the origin
        origin_scc = None
        for scc in sccs:
            if origin in scc:
                origin_scc = scc
                break

        if origin_scc is None:
            print(f"  Origin not in any SCC, expanding...")
            continue

        if destination not in origin_scc:
            print(f"  Origin and destination in different SCCs, expanding...")
            continue

        # Build subgraph from just the SCC containing both nodes
        G_scc = G_sub.subgraph(origin_scc).copy()

        # Final verification
        if not nx.has_path(G_scc, origin, destination):
            print(f"  No path in SCC subgraph, expanding...")
            continue

        # Report how much the SCC trimmed vs bounding box
        trimmed = G_sub.number_of_nodes() - G_scc.number_of_nodes()
        print(f"Subgraph valid — {G_scc.number_of_nodes()} nodes, "
              f"{G_scc.number_of_edges()} edges (padding={pad}m, "
              f"trimmed {trimmed} dead-end nodes via SCC)")
        return G_scc

    # Last resort fallback — Dijkstra path seeded subgraph
    print("Falling back to Dijkstra-seeded subgraph...")
    dijkstra_path = nx.shortest_path(G, origin, destination, weight='travel_time')
    seed_nodes = set(dijkstra_path)
    for node in dijkstra_path:
        seed_nodes.update(G.neighbors(node))
        seed_nodes.update(G.predecessors(node))

    G_sub = G.subgraph(seed_nodes).copy()

    # Apply SCC to fallback too
    sccs         = list(nx.strongly_connected_components(G_sub))
    origin_scc   = next((s for s in sccs if origin in s), None)
    if origin_scc and destination in origin_scc:
        G_sub = G_sub.subgraph(origin_scc).copy()

    print(f"Fallback subgraph — {G_sub.number_of_nodes()} nodes, "
          f"{G_sub.number_of_edges()} edges")
    return G_sub


def initialize_pheromones(G, initial_pheromone=1.0):
    for u, v, key in G.edges(keys=True):
        G[u][v][key]['pheromone'] = initial_pheromone
    return G


def get_edge_cost(G, u, v, weight='travel_time'):
    best_key  = None
    best_cost = math.inf
    for key, data in G[u][v].items():
        cost = data.get(weight, math.inf)
        if cost < best_cost:
            best_cost = cost
            best_key  = key
    return best_cost, best_key


def get_edge_pheromone(G, u, v):
    return max(data.get('pheromone', 0) for data in G[u][v].values())


def ant_tour_fast(G, origin, destination, alpha, beta,
                  edge_costs, edge_heuristics, pheromones, successors,
                  max_steps=200):
    """
    Optimized ant tour using precomputed lookups.
    max_steps caps path length to prevent excessive wandering.
    """
    current    = origin
    path       = [current]
    path_cost  = 0
    visit_counts = {origin: 1}

    for _ in range(max_steps):
        if current == destination:
            return path, path_cost

        neighbors = successors.get(current, [])
        if not neighbors:
            return None, math.inf

        scores = []
        for neighbor in neighbors:
            key       = (current, neighbor)
            pheromone = pheromones.get(key, 1e-10)
            heuristic = edge_heuristics.get(key, 1e-10)
            penalty   = 1.0 / (2.0 ** visit_counts.get(neighbor, 0))
            score     = (pheromone ** alpha) * (heuristic ** beta) * penalty
            scores.append(max(score, 1e-10))

        total         = sum(scores)
        probabilities = [s / total for s in scores]
        next_node     = random.choices(neighbors, weights=probabilities, k=1)[0]

        path_cost            += edge_costs.get((current, next_node), math.inf)
        visit_counts[next_node] = visit_counts.get(next_node, 0) + 1
        path.append(next_node)
        current = next_node

    return None, math.inf


def evaporate_pheromones_fast(pheromones, evaporation_rate):
    for key in pheromones:
        pheromones[key] = max(pheromones[key] * (1 - evaporation_rate), 1e-10)


def deposit_pheromones_fast(pheromones, path, path_cost, deposit_weight=1.0):
    if not path or path_cost == math.inf or path_cost <= 0:
        return
    deposit = deposit_weight / path_cost
    for u, v in zip(path[:-1], path[1:]):
        if (u, v) in pheromones:
            pheromones[(u, v)] += deposit


def precompute_edge_data(G, weight='travel_time'):
    """
    Cache edge costs and initial heuristics into plain dicts for O(1) lookup.
    This eliminates repeated iteration over parallel edges inside ant_tour.
    """
    edge_costs      = {}
    edge_heuristics = {}

    for u, v, data in G.edges(data=True):
        cost = data.get(weight, math.inf)

        # Keep only the minimum cost edge between each node pair
        if (u, v) not in edge_costs or cost < edge_costs[(u, v)]:
            edge_costs[(u, v)]      = cost
            edge_heuristics[(u, v)] = (1.0 / cost) if cost > 0 else 1e-10

    return edge_costs, edge_heuristics


def precompute_pheromones(G, initial_pheromone=1.0):
    """
    Store pheromones in a flat dict keyed by (u, v) instead of on the graph.
    Dict lookups are significantly faster than graph attribute access.
    """
    pheromones = {}
    for u, v in G.edges():
        pheromones[(u, v)] = initial_pheromone
    return pheromones


def precompute_successors(G):
    """
    Cache each node's successor list once so ant_tour never calls
    G.successors() repeatedly — that traverses the graph structure each time.
    """
    return {node: list(G.successors(node)) for node in G.nodes()}


def ant_colony_optimization(
    G,
    origin,
    destination,
    n_ants           = 30,
    n_iterations     = 100,
    alpha            = 1.0,
    beta             = 5.0,
    evaporation_rate = 0.15,
    deposit_weight   = 1.0,
    weight           = 'travel_time',
    padding          = 3000,
    max_steps        = 200    # hard cap on ant path length
):
    print(f"\nRunning ACO: {n_ants} ants x {n_iterations} iterations")
    print(f"alpha={alpha}, beta={beta}, evaporation={evaporation_rate}")
    print(f"max_steps={max_steps}\n")

    G_sub = build_aco_subgraph(G, origin, destination, padding)

    # Precompute everything once before the main loop
    print("Precomputing edge data...")
    edge_costs, edge_heuristics = precompute_edge_data(G_sub, weight)
    pheromones                  = precompute_pheromones(G_sub)
    successors                  = precompute_successors(G_sub)
    print("Precomputation done.\n")

    best_path = None
    best_cost = math.inf
    history   = []
    start     = time.time()

    for iteration in range(n_iterations):
        iteration_best_path = None
        iteration_best_cost = math.inf
        successful_ants     = 0

        for _ in range(n_ants):
            path, cost = ant_tour_fast(
                G_sub, origin, destination,
                alpha, beta,
                edge_costs, edge_heuristics, pheromones, successors,
                max_steps
            )

            if path and cost < math.inf:
                successful_ants += 1
                if cost < iteration_best_cost:
                    iteration_best_cost = cost
                    iteration_best_path = path
                if cost < best_cost:
                    best_cost = cost
                    best_path = path

        evaporate_pheromones_fast(pheromones, evaporation_rate)

        if iteration_best_path:
            deposit_pheromones_fast(
                pheromones, iteration_best_path,
                iteration_best_cost, deposit_weight
            )

        history.append(best_cost if best_cost < math.inf else None)

        if (iteration + 1) % 10 == 0:
            elapsed   = time.time() - start
            best_disp = f"{best_cost/60:.2f} min" if best_cost < math.inf else "none yet"
            print(f"Iteration {iteration+1:3d}/{n_iterations} | "
                  f"Best: {best_disp} | "
                  f"Successful ants: {successful_ants}/{n_ants} | "
                  f"Elapsed: {elapsed:.1f}s")

    if best_path is None:
        print("\nACO failed to find any path.")
    else:
        print(f"\nACO complete — {best_cost/60:.2f} min, {len(best_path)} nodes")

    return best_path, best_cost, history


def diagnose_aco_parameters(G, origin, destination, weight='travel_time', padding=3000):
    """
    Analyzes the graph and route characteristics to recommend
    ACO parameters tuned to the specific problem instance.
    Uses only Python standard library — no numpy required.
    """
    print("=" * 60)
    print("ACO PARAMETER DIAGNOSTICS")
    print("=" * 60)

    # --- Subgraph ---
    G_sub = build_aco_subgraph(G, origin, destination, padding)
    n_nodes = G_sub.number_of_nodes()
    n_edges = G_sub.number_of_edges()
    print(f"\nSubgraph: {n_nodes} nodes, {n_edges} edges")

    # --- Dijkstra reference path ---
    dijkstra_path = nx.shortest_path(G, origin, destination, weight=weight)
    dijkstra_cost = nx.shortest_path_length(G, origin, destination, weight=weight)
    hop_count     = len(dijkstra_path) - 1
    print(f"\nDijkstra reference:")
    print(f"  Nodes     : {len(dijkstra_path)}")
    print(f"  Hops      : {hop_count}")
    print(f"  Cost      : {dijkstra_cost/60:.2f} min")

    # --- max_steps ---
    max_steps = hop_count * 20
    print(f"\n--- max_steps ---")
    print(f"  Optimal hop count : {hop_count}")
    print(f"  Recommended       : {max_steps}  (hop_count x 20)")

    # --- Edge cost distribution (informs beta) ---
    edge_costs = [
        data.get(weight, 0)
        for u, v, data in G_sub.edges(data=True)
        if data.get(weight, 0) > 0
    ]
    cost_mean = statistics.mean(edge_costs)
    cost_std  = statistics.stdev(edge_costs)
    cost_min  = min(edge_costs)
    cost_max  = max(edge_costs)
    cost_cv   = cost_std / cost_mean  # coefficient of variation

    print(f"\n--- beta (heuristic influence) ---")
    print(f"  Edge cost mean    : {cost_mean:.2f}s")
    print(f"  Edge cost std     : {cost_std:.2f}s")
    print(f"  Edge cost min/max : {cost_min:.2f}s / {cost_max:.2f}s")
    print(f"  Coefficient of variation (CV): {cost_cv:.3f}")
    if cost_cv < 0.5:
        beta = 3
        print(f"  Low variance — edges are similar cost, beta={beta} sufficient")
    elif cost_cv < 1.0:
        beta = 5
        print(f"  Medium variance — beta={beta} recommended")
    else:
        beta = 7
        print(f"  High variance — ants benefit from strong heuristic, beta={beta}")

    # --- Branching factor (informs n_ants) ---
    successor_counts = [len(list(G_sub.successors(n))) for n in G_sub.nodes()]
    avg_branching    = statistics.mean(successor_counts)
    max_branching    = max(successor_counts)

    print(f"\n--- n_ants (exploration coverage) ---")
    print(f"  Avg branching factor : {avg_branching:.2f} successors/node")
    print(f"  Max branching factor : {max_branching}")
    # Clamps to range 20-100
    raw_n_ants = int(avg_branching * hop_count / 10) 
    n_ants     = max(20, min(100, raw_n_ants)) # I added band-aid +10 ants
    print(f"  Recommended n_ants   : {n_ants}  (branching x hops / 10, clipped 20-100)")

    # --- Reachability (informs n_iterations) ---
    reachable     = nx.single_source_shortest_path_length(
                        G_sub, origin, cutoff=hop_count * 3
                    )
    reachable_pct = len(reachable) / n_nodes

    print(f"\n--- n_iterations (convergence budget) ---")
    print(f"  Reachable nodes : {len(reachable)}/{n_nodes} ({reachable_pct*100:.1f}%)")
    if reachable_pct > 0.8:
        n_iterations = 150
        print(f"  High reachability — {n_iterations} iterations likely sufficient")
    elif reachable_pct > 0.5:
        n_iterations = 170
        print(f"  Medium reachability — {n_iterations} iterations recommended")
    else:
        n_iterations = 200
        print(f"  Low reachability — {n_iterations} iterations needed")

    # --- Evaporation rate ---
    print(f"\n--- evaporation_rate ---")
    if hop_count < 20:
        evaporation_rate = 0.3
        print(f"  Short path ({hop_count} hops) — fast evaporation={evaporation_rate}")
    elif hop_count < 50:
        evaporation_rate = 0.15
        print(f"  Medium path ({hop_count} hops) — evaporation={evaporation_rate}")
    else:
        evaporation_rate = 0.05
        print(f"  Long path ({hop_count} hops) — slow evaporation={evaporation_rate}")

    # --- deposit_weight ---
    # Scale Q to dijkstra cost magnitude so pheromone deposits
    # are numerically meaningful relative to initial pheromone of 1.0
    # Without this, deposit = 1/543 ≈ 0.0018 which evaporates before reinforcing
    deposit_weight = dijkstra_cost
    print(f"\n--- deposit_weight (Q) ---")
    print(f"  Scaled to dijkstra cost : {deposit_weight:.2f}")
    print(f"  Ensures deposit = Q/cost is in a stable numeric range")

    # --- alpha ---
    alpha = 1.0
    print(f"\n--- alpha (pheromone influence) ---")
    print(f"  Recommended : {alpha} (standard — adjust only if convergence is poor)")

    # --- Summary ---
    print(f"\n{'=' * 60}")
    print("RECOMMENDED PARAMETERS")
    print(f"{'=' * 60}")
    print(f"  n_ants           = {n_ants}")
    print(f"  n_iterations     = {n_iterations}")
    print(f"  alpha            = {alpha}")
    print(f"  beta             = {beta}")
    print(f"  evaporation_rate = {evaporation_rate}")
    print(f"  deposit_weight   = {deposit_weight:.2f}")
    print(f"  max_steps        = {max_steps}")
    print(f"{'=' * 60}\n")

    return {
        'n_ants'           : n_ants+10,
        'n_iterations'     : n_iterations,
        'alpha'            : alpha,
        'beta'             : beta,
        'evaporation_rate' : evaporation_rate,
        'deposit_weight'   : deposit_weight,
        'max_steps'        : max_steps
    }


# Used while setting up ACO -- Kept in case I want to use it again
def debug_single_ant(G, origin, destination, alpha, beta,
                     edge_costs, edge_heuristics, pheromones, successors,
                     max_steps, weight='travel_time'):
    """
    Runs one ant and prints exactly what happens step by step.
    Use this to diagnose why ants never reach the destination.
    """
    current      = origin
    path         = [current]
    path_cost    = 0
    visit_counts = {origin: 1}

    print(f"\nDebugging single ant: {origin} -> {destination}")
    print(f"Max steps: {max_steps}")

    for step in range(max_steps):
        if current == destination:
            print(f"SUCCESS at step {step}! Cost: {path_cost/60:.2f} min")
            return path, path_cost

        neighbors = successors.get(current, [])

        # Replace the stuck handling in debug_single_ant
        if not neighbors:
            print(f"STUCK at step {step} — node {current} has no successors")
            try:
                remaining = nx.shortest_path_length(G, current, destination)
                print(f"  Was {remaining} hops from destination when stuck")
            except nx.NetworkXNoPath:
                print(f"  Destination unreachable from stuck node")
                return None, math.inf

        scores = []
        for neighbor in neighbors:
            key       = (current, neighbor)
            pheromone = pheromones.get(key, 1e-10)
            heuristic = edge_heuristics.get(key, 1e-10)
            penalty   = 1.0 / (2.0 ** visit_counts.get(neighbor, 0))
            score     = (pheromone ** alpha) * (heuristic ** beta) * penalty
            scores.append(max(score, 1e-10))

        total         = sum(scores)
        probabilities = [s / total for s in scores]
        next_node     = random.choices(neighbors, weights=probabilities, k=1)[0]

        # Print every 50 steps so output stays manageable
        if step % 50 == 0:
            print(f"  Step {step:4d} | node {current} | "
                  f"{len(neighbors)} neighbors | "
                  f"visits here: {visit_counts.get(current, 0)} | "
                  f"path length: {len(path)}")

        path_cost            += edge_costs.get((current, next_node), math.inf)
        visit_counts[next_node] = visit_counts.get(next_node, 0) + 1
        path.append(next_node)
        current = next_node

    # Print final state when max_steps is hit
    print(f"TIMEOUT at step {max_steps}")
    print(f"  Final node      : {current}")
    print(f"  Path length     : {len(path)} nodes")
    print(f"  Unique nodes    : {len(set(path))}")
    print(f"  Most visited    : {max(visit_counts, key=visit_counts.get)} "
          f"({max(visit_counts.values())} times)")
    print(f"  Near destination: {nx.has_path(G, current, destination)}")

    # Check how far the ant ended up from the destination
    try:
        remaining_hops = nx.shortest_path_length(G, current, destination)
        print(f"  Hops to dest    : {remaining_hops}")
    except nx.NetworkXNoPath:
        print(f"  Hops to dest    : unreachable from final node")

    return None, math.inf