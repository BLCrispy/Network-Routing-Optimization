import math
import random
import heapq
import logging
from typing import List

import networkx as nx

from config import (
    ACO_ANTS,
    ACO_ITERATIONS,
    ACO_ALPHA,
    ACO_BETA,
    ACO_EVAP,
)

logger = logging.getLogger(__name__)


def _edge_weight(G: nx.MultiDiGraph, u: int, v: int, weight: str = "length") -> float:
    data = G.get_edge_data(u, v)
    if not data:
        return float("inf")
    return min(d.get(weight, d.get("length", float("inf"))) for d in data.values())


def _haversine(G: nx.MultiDiGraph, u: int, v: int) -> float:
    r = 6_371_000
    n1 = G.nodes[u]
    n2 = G.nodes[v]
    lat1, lon1 = math.radians(n1["y"]), math.radians(n1["x"])
    lat2, lon2 = math.radians(n2["y"]), math.radians(n2["x"])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    )
    return r * 2 * math.asin(math.sqrt(a))


def reconstruct_path(prev: dict, dest: int) -> List[int]:
    path = []
    node = dest
    while node is not None:
        path.append(node)
        node = prev.get(node)
    path.reverse()
    return path


def dijkstra(G: nx.MultiDiGraph, orig: int, dest: int, weight: str = "length") -> List[int]:
    dist = {orig: 0.0}
    prev = {orig: None}
    pq = [(0.0, orig)]

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


def astar(G: nx.MultiDiGraph, orig: int, dest: int, weight: str = "length") -> List[int]:
    def h(node_id: int) -> float:
        return _haversine(G, node_id, dest)

    g_cost = {orig: 0.0}
    prev = {orig: None}
    pq = [(h(orig), 0.0, orig)]

    while pq:
        _, current_cost, u = heapq.heappop(pq)

        if u == dest:
            return reconstruct_path(prev, dest)

        if current_cost > g_cost.get(u, float("inf")):
            continue

        for v in G.successors(u):
            w = _edge_weight(G, u, v, weight)
            ng = current_cost + w
            if ng < g_cost.get(v, float("inf")):
                g_cost[v] = ng
                prev[v] = u
                heapq.heappush(pq, (ng + h(v), ng, v))

    logger.warning("A*: no path found")
    return []


def bellman_ford(G: nx.MultiDiGraph, orig: int, dest: int, weight: str = "length") -> List[int]:
    nodes = list(G.nodes())
    dist = {n: float("inf") for n in nodes}
    prev = {n: None for n in nodes}
    dist[orig] = 0.0

    edges = []
    for u in G.nodes():
        for v in G.successors(u):
            edges.append((u, v, _edge_weight(G, u, v, weight)))

    for _ in range(len(nodes) - 1):
        updated = False
        for u, v, w in edges:
            if dist[u] != float("inf") and dist[u] + w < dist[v]:
                dist[v] = dist[u] + w
                prev[v] = u
                updated = True
        if not updated:
            break

    for u, v, w in edges:
        if dist[u] != float("inf") and dist[u] + w < dist[v]:
            logger.warning("Bellman-Ford: negative cycle detected")
            return []

    if dist[dest] == float("inf"):
        logger.warning("Bellman-Ford: no path found")
        return []

    return reconstruct_path(prev, dest)


def aco(
    G: nx.MultiDiGraph,
    orig: int,
    dest: int,
    weight: str = "length",
    n_ants: int = ACO_ANTS,
    n_iter: int = ACO_ITERATIONS,
    alpha: float = ACO_ALPHA,
    beta: float = ACO_BETA,
    evap: float = ACO_EVAP,
    q: float = 100.0,
) -> List[int]:
    tau = {}
    for u in G.nodes():
        for v in G.successors(u):
            tau[(u, v)] = 1.0

    best_path = []
    best_length = float("inf")

    for _ in range(n_iter):
        all_paths = []
        all_lengths = []

        for _ant in range(n_ants):
            path, length = _aco_walk(G, orig, dest, weight, tau, alpha, beta)
            if path:
                all_paths.append(path)
                all_lengths.append(length)
                if length < best_length:
                    best_length = length
                    best_path = path

        for key in list(tau.keys()):
            tau[key] *= (1 - evap)

        for path, length in zip(all_paths, all_lengths):
            if length <= 0:
                continue
            deposit = q / length
            for u, v in zip(path[:-1], path[1:]):
                tau[(u, v)] = tau.get((u, v), 0.0) + deposit

    if not best_path:
        logger.warning("ACO: no path found")

    return best_path


def _aco_walk(G, orig, dest, weight, tau, alpha, beta):
    current = orig
    visited = {orig}
    path = [orig]
    length = 0.0
    max_steps = len(G.nodes()) * 2

    for _ in range(max_steps):
        if current == dest:
            return path, length

        neighbors = [v for v in G.successors(current) if v not in visited]
        if not neighbors:
            return [], float("inf")

        scores = []
        for v in neighbors:
            w = _edge_weight(G, current, v, weight)
            pher = tau.get((current, v), 1.0) ** alpha
            heur = (1.0 / max(w, 1e-9)) ** beta
            scores.append(pher * heur)

        total = sum(scores)
        if total <= 0:
            return [], float("inf")

        chosen = random.choices(neighbors, weights=scores, k=1)[0]
        w = _edge_weight(G, current, chosen, weight)

        length += w
        path.append(chosen)
        visited.add(chosen)
        current = chosen

    return [], float("inf")


def bidirectional_dijkstra(
    G: nx.MultiDiGraph,
    orig: int,
    dest: int,
    weight: str = "length",
) -> List[int]:
    if orig == dest:
        return [orig]

    dist_f = {orig: 0.0}
    prev_f = {orig: None}
    pq_f = [(0.0, orig)]

    dist_b = {dest: 0.0}
    prev_b = {dest: None}
    pq_b = [(0.0, dest)]

    visited_f = set()
    visited_b = set()

    best = float("inf")
    meeting = None

    def update_meeting(node):
        nonlocal best, meeting
        if node in dist_f and node in dist_b:
            total = dist_f[node] + dist_b[node]
            if total < best:
                best = total
                meeting = node

    while pq_f or pq_b:
        if pq_f:
            d, u = heapq.heappop(pq_f)
            if u not in visited_f:
                visited_f.add(u)
                update_meeting(u)
                if d <= dist_f.get(u, float("inf")):
                    for v in G.successors(u):
                        w = _edge_weight(G, u, v, weight)
                        nd = d + w
                        if nd < dist_f.get(v, float("inf")):
                            dist_f[v] = nd
                            prev_f[v] = u
                            heapq.heappush(pq_f, (nd, v))
                            update_meeting(v)

        if pq_b:
            d, u = heapq.heappop(pq_b)
            if u not in visited_b:
                visited_b.add(u)
                update_meeting(u)
                if d <= dist_b.get(u, float("inf")):
                    for v in G.predecessors(u):
                        w = _edge_weight(G, v, u, weight)
                        nd = d + w
                        if nd < dist_b.get(v, float("inf")):
                            dist_b[v] = nd
                            prev_b[v] = u
                            heapq.heappush(pq_b, (nd, v))
                            update_meeting(v)

        top_f = pq_f[0][0] if pq_f else float("inf")
        top_b = pq_b[0][0] if pq_b else float("inf")
        if meeting is not None and top_f + top_b >= best:
            break

    if meeting is None:
        logger.warning("Bidirectional Dijkstra: no path found")
        return []

    return _merge_paths(prev_f, prev_b, meeting)


def bidirectional_astar(
    G: nx.MultiDiGraph,
    orig: int,
    dest: int,
    weight: str = "length",
) -> List[int]:
    if orig == dest:
        return [orig]

    def h_forward(node):
        return _haversine(G, node, dest)

    def h_backward(node):
        return _haversine(G, node, orig)

    g_f = {orig: 0.0}
    prev_f = {orig: None}
    pq_f = [(h_forward(orig), 0.0, orig)]

    g_b = {dest: 0.0}
    prev_b = {dest: None}
    pq_b = [(h_backward(dest), 0.0, dest)]

    visited_f = set()
    visited_b = set()

    best = float("inf")
    meeting = None

    def update_meeting(node):
        nonlocal best, meeting
        if node in g_f and node in g_b:
            total = g_f[node] + g_b[node]
            if total < best:
                best = total
                meeting = node

    while pq_f or pq_b:
        if pq_f:
            _, cost, u = heapq.heappop(pq_f)
            if u not in visited_f:
                visited_f.add(u)
                update_meeting(u)
                if cost <= g_f.get(u, float("inf")):
                    for v in G.successors(u):
                        w = _edge_weight(G, u, v, weight)
                        ng = cost + w
                        if ng < g_f.get(v, float("inf")):
                            g_f[v] = ng
                            prev_f[v] = u
                            heapq.heappush(pq_f, (ng + h_forward(v), ng, v))
                            update_meeting(v)

        if pq_b:
            _, cost, u = heapq.heappop(pq_b)
            if u not in visited_b:
                visited_b.add(u)
                update_meeting(u)
                if cost <= g_b.get(u, float("inf")):
                    for v in G.predecessors(u):
                        w = _edge_weight(G, v, u, weight)
                        ng = cost + w
                        if ng < g_b.get(v, float("inf")):
                            g_b[v] = ng
                            prev_b[v] = u
                            heapq.heappush(pq_b, (ng + h_backward(v), ng, v))
                            update_meeting(v)

        top_f = pq_f[0][1] if pq_f else float("inf")
        top_b = pq_b[0][1] if pq_b else float("inf")
        if meeting is not None and top_f + top_b >= best:
            break

    if meeting is None:
        logger.warning("Bidirectional A*: no path found")
        return []

    return _merge_paths(prev_f, prev_b, meeting)


def _merge_paths(prev_f: dict, prev_b: dict, meeting: int) -> List[int]:
    path_f = []
    node = meeting
    while node is not None:
        path_f.append(node)
        node = prev_f.get(node)
    path_f.reverse()

    path_b = []
    node = prev_b.get(meeting)
    while node is not None:
        path_b.append(node)
        node = prev_b.get(node)

    return path_f + path_b


ALGO_MAP = {
    "dijkstra": dijkstra,
    "a*": astar,
    "bellman-ford": bellman_ford,
    "aco": aco,
    "bidirectional dijkstra": bidirectional_dijkstra,
    "bidirectional a*": bidirectional_astar,
}


def find_path(
    G: nx.MultiDiGraph,
    orig: int,
    dest: int,
    algorithm: str = "A*",
    weight: str = "length",
) -> List[int]:
    label_map = {
        "bi-dijkstra": "bidirectional dijkstra",
        "bi-a*": "bidirectional a*",
        "bidirectional dijkstra": "bidirectional dijkstra",
        "bidirectional a*": "bidirectional a*",
    }

    key = algorithm.lower()
    key = label_map.get(key, key)

    fn = ALGO_MAP.get(key)
    if fn is None:
        raise ValueError(f"Unknown algorithm: {algorithm}")

    if G is None:
        return []

    if orig not in G or dest not in G:
        logger.warning("find_path: origin or destination node missing from graph")
        return []

    if orig == dest:
        return [orig]

    return fn(G, orig, dest, weight=weight)

