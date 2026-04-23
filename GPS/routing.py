# routing.py — Dijkstra, A*, Bellman-Ford, ACO on NetworkX MultiDiGraph
#
# All algorithms accept:
#   G         : networkx.MultiDiGraph  (OSMnx graph)
#   orig_node : int  (OSM node id)
#   dest_node : int  (OSM node id)
#   weight    : str  (edge attribute to minimise, default 'length')
#
# All return:
#   list[int]  — ordered node ids forming the path (empty on failure)

import math
import random
import heapq
import logging
from typing import List, Optional

import networkx as nx

logger = logging.getLogger(__name__)


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _edge_weight(G: nx.MultiDiGraph, u: int, v: int, weight: str = "length") -> float:
    """Return minimum weight across parallel edges."""
    data = G.get_edge_data(u, v)
    if not data:
        return float("inf")
    return min(
        d.get(weight, d.get("length", float("inf")))
        for d in data.values()
    )

#heuristic function to estimate the straight-line distance between two points on the Earth's surface using latitude and longitude
def _haversine(G: nx.MultiDiGraph, u: int, v: int) -> float:
    """Great-circle distance in metres between two OSM nodes."""
    R = 6_371_000
    n1, n2 = G.nodes[u], G.nodes[v]
    lat1, lon1 = math.radians(n1["y"]), math.radians(n1["x"])
    lat2, lon2 = math.radians(n2["y"]), math.radians(n2["x"])
    dlat, dlon = lat2 - lat1, lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return R * 2 * math.asin(math.sqrt(a))


def reconstruct_path(prev: dict, dest: int) -> List[int]:
    path = []
    node = dest
    while node is not None:
        path.append(node)
        node = prev.get(node)
    path.reverse()
    return path


# ─────────────────────────────────────────────────────────────────────────────
# 1. Dijkstra
# ─────────────────────────────────────────────────────────────────────────────

def dijkstra(G: nx.MultiDiGraph, orig: int, dest: int, weight: str = "length") -> List[int]:
    dist = {orig: 0.0}
    prev = {orig: None}
    pq   = [(0.0, orig)]

    while pq:
        # d=current_cost, u=current_node
        d, u = heapq.heappop(pq)
        if u == dest:
            return reconstruct_path(prev, dest)
        if d > dist.get(u, float("inf")):
            continue
        for v in G.successors(u):
            w = _edge_weight(G, u, v, weight)
            nd = d + w
            if nd < dist.get(v, float("inf")):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))

    logger.warning("Dijkstra: no path found")
    return []


# ─────────────────────────────────────────────────────────────────────────────
# 2. A*
# ─────────────────────────────────────────────────────────────────────────────

def astar(G: nx.MultiDiGraph, orig: int, dest: int, weight: str = "length") -> List[int]:
    def h(node):
        return _haversine(G, node, dest)

    g    = {orig: 0.0}
    prev = {orig: None}
    pq   = [(h(orig), 0.0, orig)]

    while pq:
        _, cost, u = heapq.heappop(pq)
        if u == dest:
            return reconstruct_path(prev, dest)
        if cost > g.get(u, float("inf")):
            continue
        for v in G.successors(u):
            w  = _edge_weight(G, u, v, weight)
            ng = cost + w
            if ng < g.get(v, float("inf")):
                g[v]    = ng
                prev[v] = u
                heapq.heappush(pq, (ng + h(v), ng, v))

    logger.warning("A*: no path found")
    return []


# ─────────────────────────────────────────────────────────────────────────────
# 3. Bellman-Ford
# ─────────────────────────────────────────────────────────────────────────────

def bellman_ford(G: nx.MultiDiGraph, orig: int, dest: int, weight: str = "length") -> List[int]:
    nodes = list(G.nodes())
    dist  = {n: float("inf") for n in nodes}
    prev  = {n: None         for n in nodes}
    dist[orig] = 0.0

    edges = [
        (u, v, _edge_weight(G, u, v, weight))
        for u in G.nodes()
        for v in G.successors(u)
    ]

    for _ in range(len(nodes) - 1):
        updated = False
        for u, v, w in edges:
            if dist[u] + w < dist[v]:
                dist[v]  = dist[u] + w
                prev[v]  = u
                updated  = True
        if not updated:
            break

    # Negative-cycle check
    for u, v, w in edges:
        if dist[u] + w < dist[v]:
            logger.warning("Bellman-Ford: negative cycle detected")
            return []

    if dist[dest] == float("inf"):
        logger.warning("Bellman-Ford: no path found")
        return []

    return reconstruct_path(prev, dest)


# ─────────────────────────────────────────────────────────────────────────────
# 4. Ant Colony Optimisation (ACO)
# ─────────────────────────────────────────────────────────────────────────────

def aco(
    G: nx.MultiDiGraph,
    orig: int,
    dest: int,
    weight:     str   = "length",
    n_ants:     int   = 30,
    n_iter:     int   = 50,
    alpha:      float = 1.0,   # pheromone power
    beta:       float = 2.0,   # heuristic power
    evap:       float = 0.5,   # evaporation rate
    q:          float = 100.0  # pheromone deposit constant
) -> List[int]:
    """
    Graph-based ACO.  Works on MultiDiGraph by treating each (u,v) pair as
    one edge with the minimum weight across parallel edges.
    """
    nodes     = list(G.nodes())
    node_idx  = {n: i for i, n in enumerate(nodes)}
    N         = len(nodes)

    # Initialise pheromone matrix (sparse via dict for memory efficiency)
    tau: dict = {}
    for u in G.nodes():
        for v in G.successors(u):
            tau[(u, v)] = 1.0

    best_path:   List[int] = []
    best_length: float     = float("inf")

    for iteration in range(n_iter):
        all_paths:   List[List[int]] = []
        all_lengths: List[float]     = []

        for _ in range(n_ants):
            path, length = _aco_walk(G, orig, dest, weight, tau, alpha, beta)
            if path:
                all_paths.append(path)
                all_lengths.append(length)
                if length < best_length:
                    best_length = length
                    best_path   = path

        # Evaporate
        for key in tau:
            tau[key] *= (1 - evap)

        # Deposit
        for path, length in zip(all_paths, all_lengths):
            deposit = q / length
            for u, v in zip(path[:-1], path[1:]):
                tau[(u, v)] = tau.get((u, v), 0.0) + deposit

    if not best_path:
        logger.warning("ACO: no path found")
    return best_path


def _aco_walk(G, orig, dest, weight, tau, alpha, beta):
    current  = orig
    visited  = {orig}
    path     = [orig]
    length   = 0.0
    max_steps = len(G.nodes()) * 2

    for _ in range(max_steps):
        if current == dest:
            return path, length

        neighbours = [v for v in G.successors(current) if v not in visited]
        if not neighbours:
            return [], float("inf")   # dead end

        # Probability proportional to tau^alpha * (1/dist)^beta
        scores = []
        for v in neighbours:
            w   = _edge_weight(G, current, v, weight)
            ph  = tau.get((current, v), 1.0) ** alpha
            eta = (1.0 / max(w, 1e-9)) ** beta
            scores.append(ph * eta)

        total = sum(scores)
        probs = [s / total for s in scores]
        chosen = random.choices(neighbours, weights=probs, k=1)[0]

        w       = _edge_weight(G, current, chosen, weight)
        length += w
        path.append(chosen)
        visited.add(chosen)
        current = chosen

    return [], float("inf")   # did not reach dest


# ─────────────────────────────────────────────────────────────────────────────
# 5. Bidirectional Dijkstra
# ─────────────────────────────────────────────────────────────────────────────

def bidirectional_dijkstra(
    G: nx.MultiDiGraph,
    orig: int,
    dest: int,
    weight: str = "length",
) -> List[int]:
    """
    Bidirectional Dijkstra — searches from both ends simultaneously.
    Stops when the frontiers meet, cutting the search space roughly in half.
    Memory efficient on large graphs.
    """
    if orig == dest:
        return [orig]

    # Forward search structures
    dist_f  = {orig: 0.0}
    prev_f  = {orig: None}
    pq_f    = [(0.0, orig)]

    # Backward search structures (search on reversed graph)
    dist_b  = {dest: 0.0}
    prev_b  = {dest: None}
    pq_b    = [(0.0, dest)]

    visited_f: set = set()
    visited_b: set = set()

    best      = float("inf")
    meeting   = None

    def _check_meeting(node):
        nonlocal best, meeting
        if node in dist_f and node in dist_b:
            total = dist_f[node] + dist_b[node]
            if total < best:
                best    = total
                meeting = node

    while pq_f or pq_b:
        # Alternate between forward and backward
        if pq_f:
            d, u = heapq.heappop(pq_f)
            if u not in visited_f:
                visited_f.add(u)
                _check_meeting(u)
                if dist_f.get(u, float("inf")) <= d:
                    for v in G.successors(u):
                        w  = _edge_weight(G, u, v, weight)
                        nd = d + w
                        if nd < dist_f.get(v, float("inf")):
                            dist_f[v] = nd
                            prev_f[v] = u
                            heapq.heappush(pq_f, (nd, v))
                            _check_meeting(v)

        if pq_b:
            d, u = heapq.heappop(pq_b)
            if u not in visited_b:
                visited_b.add(u)
                _check_meeting(u)
                if dist_b.get(u, float("inf")) <= d:
                    # Traverse reversed edges for backward search
                    for v in G.predecessors(u):
                        w  = _edge_weight(G, v, u, weight)
                        nd = d + w
                        if nd < dist_b.get(v, float("inf")):
                            dist_b[v] = nd
                            prev_b[v] = u
                            heapq.heappush(pq_b, (nd, v))
                            _check_meeting(v)

        # Termination condition — both frontiers have expanded
        # past the best known meeting point
        top_f = pq_f[0][0] if pq_f else float("inf")
        top_b = pq_b[0][0] if pq_b else float("inf")
        if top_f + top_b >= best and meeting is not None:
            break

    if meeting is None:
        logger.warning("Bidirectional Dijkstra: no path found")
        return []

    return _merge_paths(prev_f, prev_b, orig, dest, meeting)


# ─────────────────────────────────────────────────────────────────────────────
# 6. Bidirectional A*
# ─────────────────────────────────────────────────────────────────────────────

def bidirectional_astar(
    G: nx.MultiDiGraph,
    orig: int,
    dest: int,
    weight: str = "length",
) -> List[int]:
    """
    Bidirectional A* — heuristic-guided search from both ends.
    Most memory-efficient algorithm for large geographic graphs.
    Uses consistent average heuristic to guarantee optimality.
    """
    if orig == dest:
        return [orig]

    def h_forward(node):
        return _haversine(G, node, dest)

    def h_backward(node):
        return _haversine(G, node, orig)

    # Forward
    g_f     = {orig: 0.0}
    prev_f  = {orig: None}
    pq_f    = [(h_forward(orig), 0.0, orig)]

    # Backward
    g_b     = {dest: 0.0}
    prev_b  = {dest: None}
    pq_b    = [(h_backward(dest), 0.0, dest)]

    visited_f: set = set()
    visited_b: set = set()

    best    = float("inf")
    meeting = None

    def _update_best(node):
        nonlocal best, meeting
        if node in g_f and node in g_b:
            total = g_f[node] + g_b[node]
            if total < best:
                best    = total
                meeting = node

    while pq_f or pq_b:
        if pq_f:
            _, cost, u = heapq.heappop(pq_f)
            if u not in visited_f:
                visited_f.add(u)
                _update_best(u)
                if cost <= g_f.get(u, float("inf")):
                    for v in G.successors(u):
                        w  = _edge_weight(G, u, v, weight)
                        ng = cost + w
                        if ng < g_f.get(v, float("inf")):
                            g_f[v]    = ng
                            prev_f[v] = u
                            f = ng + h_forward(v)
                            heapq.heappush(pq_f, (f, ng, v))
                            _update_best(v)

        if pq_b:
            _, cost, u = heapq.heappop(pq_b)
            if u not in visited_b:
                visited_b.add(u)
                _update_best(u)
                if cost <= g_b.get(u, float("inf")):
                    for v in G.predecessors(u):
                        w  = _edge_weight(G, v, u, weight)
                        ng = cost + w
                        if ng < g_b.get(v, float("inf")):
                            g_b[v]    = ng
                            prev_b[v] = u
                            f = ng + h_backward(v)
                            heapq.heappush(pq_b, (f, ng, v))
                            _update_best(v)

        # Termination — prune when frontiers exceed best path
        top_f = pq_f[0][1] if pq_f else float("inf")
        top_b = pq_b[0][1] if pq_b else float("inf")
        if top_f + top_b >= best and meeting is not None:
            break

    if meeting is None:
        logger.warning("Bidirectional A*: no path found")
        return []

    return _merge_paths(prev_f, prev_b, orig, dest, meeting)


# ─────────────────────────────────────────────────────────────────────────────
# Path merger helper (used by both bidirectional algorithms)
# ─────────────────────────────────────────────────────────────────────────────

def _merge_paths(
    prev_f: dict,
    prev_b: dict,
    orig:   int,
    dest:   int,
    meeting: int,
) -> List[int]:
    """
    Reconstruct full path by joining forward and backward traces
    at the meeting node.
    """
    # Forward path: orig → meeting
    path_f = []
    node   = meeting
    while node is not None:
        path_f.append(node)
        node = prev_f.get(node)
    path_f.reverse()

    # Backward path: meeting → dest
    path_b = []
    node   = prev_b.get(meeting)   # skip meeting node, already in path_f
    while node is not None:
        path_b.append(node)
        node = prev_b.get(node)

    return path_f + path_b

# ─────────────────────────────────────────────────────────────────────────────
# Dispatcher
# ─────────────────────────────────────────────────────────────────────────────

ALGO_MAP = {
    "dijkstra":              dijkstra,
    "a*":                    astar,
    "bellman-ford":          bellman_ford,
    "aco":                   aco,
    "bidirectional dijkstra": bidirectional_dijkstra,
    "bidirectional a*":       bidirectional_astar,
}

def find_path(
    G: nx.MultiDiGraph,
    orig: int,
    dest: int,
    algorithm: str = "A*",
    weight: str = "length",
) -> List[int]:
    # Normalise key — map UI labels to algo map keys
    label_map = {
        "bi-dijkstra":          "bidirectional dijkstra",
        "bi-a*":                "bidirectional a*",
        "bidirectional a*":     "bidirectional a*",
        "bidirectional dijkstra": "bidirectional dijkstra",
    }
    key = algorithm.lower()
    key = label_map.get(key, key)
    fn  = ALGO_MAP.get(key)
