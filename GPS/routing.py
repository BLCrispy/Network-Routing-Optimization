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
# Dispatcher
# ─────────────────────────────────────────────────────────────────────────────

ALGO_MAP = {
    "dijkstra":     dijkstra,
    "a*":           astar,
    "bellman-ford": bellman_ford,
    "aco":          aco,
}


def find_path(
    G: nx.MultiDiGraph,
    orig: int,
    dest: int,
    algorithm: str = "A*",
    weight: str = "length",
) -> List[int]:
    """
    Unified entry point.
    algorithm: one of 'A*', 'Dijkstra', 'Bellman-Ford', 'ACO'  (case-insensitive)
    """
    key = algorithm.lower()
    fn  = ALGO_MAP.get(key)
    if fn is None:
        logger.error(f"Unknown algorithm '{algorithm}', falling back to A*")
        fn = astar
    try:
        return fn(G, orig, dest, weight)
    except Exception as e:
        logger.error(f"Routing error ({algorithm}): {e}")
        return []
