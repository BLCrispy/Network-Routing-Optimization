"""
US OSM Graph Benchmark — benchmark_us.py
=========================================
Downloads a pre-stripped US road network, generates 250 subgraphs of
progressively increasing size (≈1 000 nodes → full US), runs Dijkstra,
A*, Bellman-Ford, and ACO on every subgraph, and writes results to a CSV.

SETUP — where to get the graph
───────────────────────────────
We use the Geofabrik North America extract (filtered to highways only).
This is the lightest US-scale dataset that still has all OSM attributes.

    1.  Go to:  https://download.geofabrik.de/north-america/us.html
    2.  Download "us-latest.osm.pbf"  (≈ 8–9 GB compressed)
    3.  Filter it to drive-able roads only with osmium (saves RAM):
            osmium tags-filter us-latest.osm.pbf \
                w/highway=motorway,trunk,primary,secondary,tertiary,unclassified,residential \
                -o us_roads.osm.pbf --overwrite
        Install osmium:  sudo apt install osmium-tool  (Linux)
                         brew install osmium-tool       (Mac)
                         https://osmcode.org/osmium-tool/ (Windows)
    4.  Place  us_roads.osm.pbf  in the same directory as this script.

OSMnx can then load it with:
    G = ox.graph_from_xml('us_roads.osm.pbf', retain_all=False,
                          network_type='drive')

Expected size after filtering: ≈ 50–80 M nodes, ≈ 120 M edges (projected).
The script works on the projected (metre-coordinate) version so A* heuristics
are in metres and travel_time can be added with ox.add_edge_travel_times().

Memory budget assumed: 16 GB RAM total, ~12 GB available to this process.

HOW SUBGRAPHS ARE BUILT
────────────────────────
250 subgraphs are built by ego-graph expansion from a fixed "centre" node
near the geographic centroid of the continental US (≈ Kansas).  Radius is
increased linearly so node counts grow from ≈ 1 000 to the full graph size.

For subgraphs large enough to span the US, a random long-distance origin/
destination pair is chosen (sampled from opposite quadrants of the bounding
box) so that path-finding actually tests the algorithm across the whole graph.

RESOURCE LIMITS
────────────────
  Timeout  : 10 minutes per algorithm per graph
  Memory   : 16 GB ceiling; algorithm is killed if it exceeds this

OUTPUT
───────
  src/results/benchmark_results.csv
Graphs_Archive/scalability_path_benchmark/
      graph_001_Kansas_1000n.png
      graph_002_Kansas_1243n.png
      … (one PNG per subgraph, saved in a background thread so the
         benchmark loop never waits for matplotlib)
"""

import os
import sys
import csv
import gc
import math
import pickle
import time
import random
import signal
import traceback
import threading
import statistics
from pathlib import Path

import queue

import psutil
import matplotlib
matplotlib.use("Agg")          # non-interactive backend — never opens a window
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
import osmnx as ox
import networkx as nx

# ── locate myAlgorithms.py ────────────────────────────────────────────────────
sys.path.insert(0, str(Path(__file__).parent))
from myAlgorithms import (
    dijkstra,
    astar,
    bellman_ford,
    ant_colony_optimization,
    diagnose_aco_parameters,
)

# ═══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════════

PBF_PATH          = "us_roads.osm.pbf"   # filtered OSM PBF — see header

# ── Directory layout (mirrors the repo structure) ─────────────────────────────
# This script lives in  <repo>/src/
# Graphs_Archive sits at <repo>/Graphs_Archive/   (same level as src/)
_SRC_DIR          = Path(__file__).parent.resolve()
_REPO_ROOT        = _SRC_DIR.parent
OUTPUT_DIR        = _SRC_DIR / "results"
PLOT_DIR          = _REPO_ROOT / "Graphs_Archive" / "scalability_path_benchmark"

OUTPUT_CSV        = OUTPUT_DIR / "benchmark_results.csv"
NUM_SUBGRAPHS     = 150
MIN_NODES         = 1_000
MAX_GRAPH_NODES   = 10_000_000
ACO_MAX_NODES     = 100_000
TIMEOUT_SECONDS   = 600          # 10 minutes
MEMORY_LIMIT_GB   = 16.0
WEIGHT            = "travel_time"
RANDOM_SEED       = 42
SUBSET_CENTER_LAT = 39.8283      # continental US center-ish (Kansas)
SUBSET_CENTER_LON = -98.5795
RADIUS_BIN_KM     = 25
RADIUS_TARGET_BUFFER = 1.10
GRAPH_CACHE_NAME  = (
    f"us_roads_benchmark_graph_{MAX_GRAPH_NODES // 1_000_000}m_nodes_"
    f"center_subset.pkl"
)
GRAPH_CRS         = "EPSG:5070"
QUADRANT_SAMPLE_LIMIT = 50_000

random.seed(RANDOM_SEED)

# ═══════════════════════════════════════════════════════════════════════════════
# RESOURCE MONITORING
# ═══════════════════════════════════════════════════════════════════════════════

_PROCESS = psutil.Process(os.getpid())

def current_memory_gb() -> float:
    """RSS memory of this process in GB."""
    return _PROCESS.memory_info().rss / (1024 ** 3)


class TimeoutError(Exception):
    pass


class MemoryError(Exception):
    pass


def _memory_watcher(stop_event: threading.Event,
                    limit_gb: float,
                    triggered: list):
    """Background thread that sets triggered[0]=True when RAM exceeds limit."""
    while not stop_event.is_set():
        if current_memory_gb() >= limit_gb:
            triggered[0] = True
            return
        time.sleep(0.5)


# ═══════════════════════════════════════════════════════════════════════════════
# GRAPH LOADING
# ═══════════════════════════════════════════════════════════════════════════════

_DRIVE_HIGHWAYS = {
    "motorway", "trunk", "primary", "secondary", "tertiary",
    "unclassified", "residential",
    "motorway_link", "trunk_link", "primary_link",
    "secondary_link", "tertiary_link",
}

_ONEWAY_TRUE = {"yes", "true", "1", "t"}
_ONEWAY_FALSE = {"no", "false", "0", "f"}
_MPH_TO_KPH = 1.60934
_DEFAULT_SPEED_KPH = {
    "motorway": 105.0,
    "motorway_link": 65.0,
    "trunk": 95.0,
    "trunk_link": 55.0,
    "primary": 80.0,
    "primary_link": 50.0,
    "secondary": 65.0,
    "secondary_link": 45.0,
    "tertiary": 50.0,
    "tertiary_link": 40.0,
    "unclassified": 40.0,
    "residential": 30.0,
}


def _graph_cache_path(pbf_path: Path) -> Path:
    return pbf_path.with_name(GRAPH_CACHE_NAME)


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Fast great-circle distance in metres."""
    r = 6_371_000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lam = math.radians(lon2 - lon1)
    a = (
        math.sin(d_phi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(d_lam / 2.0) ** 2
    )
    return 2.0 * r * math.asin(math.sqrt(a))


def _parse_maxspeed_kph(value) -> float | None:
    """Parse common OSM maxspeed formats into km/h."""
    if value is None:
        return None

    if isinstance(value, (list, tuple)):
        for item in value:
            parsed = _parse_maxspeed_kph(item)
            if parsed is not None:
                return parsed
        return None

    text = str(value).strip().lower()
    if not text:
        return None

    text = text.split(";", 1)[0].strip()
    if text in {"signals", "none", "walk", "variable"}:
        return None

    parts = text.replace(",", ".").split()
    try:
        number = float(parts[0])
    except (IndexError, ValueError):
        return None

    if "mph" in text:
        return number * _MPH_TO_KPH
    return number


def _edge_speed_kph(tags) -> float:
    """Infer a usable speed when maxspeed is absent."""
    highway = tags.get("highway", "")
    return (
        _parse_maxspeed_kph(tags.get("maxspeed"))
        or _DEFAULT_SPEED_KPH.get(highway, 40.0)
    )


def _way_direction_flags(tags) -> tuple[bool, bool]:
    """
    Return (forward_allowed, reverse_allowed) for a drivable way.
    """
    oneway = str(tags.get("oneway", "")).strip().lower()
    junction = str(tags.get("junction", "")).strip().lower()
    highway = tags.get("highway", "")

    if oneway == "-1":
        return False, True
    if oneway in _ONEWAY_TRUE or junction == "roundabout":
        return True, False
    if oneway in _ONEWAY_FALSE:
        return True, True
    if highway in {"motorway", "motorway_link"}:
        return True, False
    return True, True


def _project_graph_nodes(G):
    """
    Project lon/lat node coordinates into a continental-US metric CRS.
    """
    try:
        from pyproj import Transformer
    except ImportError:
        print("  pyproj unavailable; falling back to OSMnx graph projection …")
        original_coords = {
            node: (data["lon"], data["lat"])
            for node, data in G.nodes(data=True)
        }
        G.graph["crs"] = "EPSG:4326"
        G = ox.project_graph(G, to_crs=GRAPH_CRS)
        for node, data in G.nodes(data=True):
            lon, lat = original_coords[node]
            data["lon"] = lon
            data["lat"] = lat
        return G

    transformer = Transformer.from_crs("EPSG:4326", GRAPH_CRS, always_xy=True)
    for _, data in G.nodes(data=True):
        lon = data["lon"]
        lat = data["lat"]
        data["x"], data["y"] = transformer.transform(lon, lat)
    G.graph["crs"] = GRAPH_CRS
    return G


def _simplify_benchmark_graph(G):
    """
    Remove the giant component noise and collapse straight-through nodes so
    the benchmark keeps only routing-relevant structure.
    """
    if G.number_of_nodes() == 0:
        return G

    print("  Keeping the largest weakly connected component …")
    largest_wcc = max(nx.weakly_connected_components(G), key=len)
    G = G.subgraph(largest_wcc).copy()

    print("  Simplifying topology …")
    G = ox.simplification.simplify_graph(
        G,
        edge_attr_aggs={
            "length": sum,
            "travel_time": sum,
        },
    )

    # Keep edge attributes lightweight and numeric after aggregation.
    for _, _, data in G.edges(data=True):
        if isinstance(data.get("length"), list):
            data["length"] = float(sum(data["length"]))
        if isinstance(data.get("travel_time"), list):
            data["travel_time"] = float(sum(data["travel_time"]))
        if isinstance(data.get("speed_kph"), list):
            vals = [v for v in data["speed_kph"] if isinstance(v, (int, float))]
            data["speed_kph"] = float(statistics.mean(vals)) if vals else None
        data.pop("geometry", None)

    return G


def _load_cached_graph(cache_path: Path):
    with cache_path.open("rb") as fh:
        G = pickle.load(fh)
    print(
        f"  Loaded cached graph: {G.number_of_nodes():,} nodes, "
        f"{G.number_of_edges():,} edges"
    )
    return G


def _save_cached_graph(G, cache_path: Path):
    cache_path.parent.mkdir(parents=True, exist_ok=True)
    with cache_path.open("wb") as fh:
        pickle.dump(G, fh, protocol=pickle.HIGHEST_PROTOCOL)
    size_gb = cache_path.stat().st_size / (1024 ** 3)
    print(f"  Cached processed graph → {cache_path}  ({size_gb:.2f} GB)")


def _estimate_subset_radius_m(pbf_path: Path) -> float:
    """
    Scan the PBF once to estimate a center-out radius that should keep roughly
    MAX_GRAPH_NODES worth of road nodes near the continental-US center.
    """
    try:
        import osmium
    except ImportError as exc:
        raise ImportError(
            "Radius estimation requires the 'osmium' package. "
            "Install it with 'pip install osmium' or "
            "'conda install -c conda-forge osmium-tool'."
        ) from exc

    bin_m = RADIUS_BIN_KM * 1000.0
    target = int(MAX_GRAPH_NODES * RADIUS_TARGET_BUFFER)

    class _RadiusEstimator(osmium.SimpleHandler):
        def __init__(self):
            super().__init__()
            self.bin_counts = {}
            self.node_samples = 0

        def way(self, w):
            highway = w.tags.get("highway", "")
            if highway not in _DRIVE_HIGHWAYS:
                return

            for node in w.nodes:
                if not node.location.valid():
                    continue

                dist_m = _haversine_m(
                    SUBSET_CENTER_LAT,
                    SUBSET_CENTER_LON,
                    node.location.lat,
                    node.location.lon,
                )
                bin_idx = int(dist_m // bin_m)
                self.bin_counts[bin_idx] = self.bin_counts.get(bin_idx, 0) + 1
                self.node_samples += 1

    estimator = _RadiusEstimator()
    location_indexes = [
        "sparse_mmap_array",
        "sparse_file_array",
        "flex_mem",
    ]
    last_error = None

    print(
        "Estimating geographic subset radius from the US center "
        f"({SUBSET_CENTER_LAT:.4f}, {SUBSET_CENTER_LON:.4f}) …"
    )
    for idx_name in location_indexes:
        try:
            print(f"  osmium node-location index: {idx_name} [radius-estimate]")
            estimator.apply_file(str(pbf_path), locations=True, idx=idx_name)
            last_error = None
            break
        except Exception as exc:
            last_error = exc
            estimator = _RadiusEstimator()

    if last_error is not None:
        raise last_error

    if not estimator.bin_counts:
        raise RuntimeError("Radius estimation found no drivable road nodes.")

    running = 0
    selected_bin = max(estimator.bin_counts)
    for bin_idx in sorted(estimator.bin_counts):
        running += estimator.bin_counts[bin_idx]
        if running >= target:
            selected_bin = bin_idx
            break

    radius_m = (selected_bin + 1) * bin_m
    print(
        f"  Estimated radius: {radius_m / 1000:.0f} km "
        f"for ~{running:,} sampled road-node touches"
    )
    return radius_m


def _build_graph_from_pbf(pbf_path: Path):
    """
    Stream the PBF directly with osmium, build only the attributes the
    benchmark needs inside a geographic center-out radius, stop once the
    configured node cap is reached, then simplify and project once.
    """
    try:
        import osmium
    except ImportError as exc:
        raise ImportError(
            "Direct PBF loading requires the 'osmium' package. "
            "Install it with 'pip install osmium' or "
            "'conda install -c conda-forge osmium-tool'."
        ) from exc

    class _ReachedNodeCap(Exception):
        """Raised to stop the osmium stream once the node cap is reached."""
        pass

    subset_radius_m = _estimate_subset_radius_m(pbf_path)

    class _RoadGraphBuilder(osmium.SimpleHandler):
        def __init__(self, allowed_radius_m: float):
            super().__init__()
            self.G = nx.MultiDiGraph()
            self.allowed_radius_m = allowed_radius_m
            self._node_count = 0
            self._way_count = 0
            self._edge_count = 0
            self._hit_node_cap = False

        def way(self, w):
            highway = w.tags.get("highway", "")
            if highway not in _DRIVE_HIGHWAYS:
                return

            forward_ok, reverse_ok = _way_direction_flags(w.tags)
            speed_kph = _edge_speed_kph(w.tags)
            speed_mps = max(speed_kph * 1000.0 / 3600.0, 0.1)

            coords = []
            for node in w.nodes:
                if not node.location.valid():
                    return
                coords.append((node.ref, node.location.lon, node.location.lat))

            if len(coords) < 2:
                return

            self._way_count += 1

            for (u, lon_u, lat_u), (v, lon_v, lat_v) in zip(coords, coords[1:]):
                u_dist_m = _haversine_m(SUBSET_CENTER_LAT, SUBSET_CENTER_LON, lat_u, lon_u)
                v_dist_m = _haversine_m(SUBSET_CENTER_LAT, SUBSET_CENTER_LON, lat_v, lon_v)
                if u_dist_m > self.allowed_radius_m or v_dist_m > self.allowed_radius_m:
                    continue

                length_m = _haversine_m(lat_u, lon_u, lat_v, lon_v)
                if length_m <= 0:
                    continue

                u_is_new = u not in self.G
                v_is_new = v not in self.G
                nodes_to_add = int(u_is_new) + int(v_is_new and v != u)
                if self._node_count + nodes_to_add > MAX_GRAPH_NODES:
                    self._hit_node_cap = True
                    raise _ReachedNodeCap

                if u_is_new:
                    self.G.add_node(u, lon=lon_u, lat=lat_u, x=lon_u, y=lat_u)
                    self._node_count += 1
                if v_is_new and v != u:
                    self.G.add_node(v, lon=lon_v, lat=lat_v, x=lon_v, y=lat_v)
                    self._node_count += 1

                edge_attrs = {
                    "length": float(length_m),
                    "travel_time": float(length_m / speed_mps),
                    "speed_kph": float(speed_kph),
                    "highway": highway,
                }

                if forward_ok:
                    self.G.add_edge(u, v, **edge_attrs)
                    self._edge_count += 1
                if reverse_ok:
                    self.G.add_edge(v, u, **edge_attrs)
                    self._edge_count += 1

                if self._edge_count and self._edge_count % 1_000_000 == 0:
                    print(
                        f"    streamed {self._edge_count:,} directed edges | "
                        f"RAM {current_memory_gb():.1f} GB"
                    )

    print(f"Loading OSM graph directly from {pbf_path} …")
    t0 = time.time()
    handler = _RoadGraphBuilder(subset_radius_m)
    location_indexes = [
        "sparse_mmap_array",
        "sparse_file_array",
        "flex_mem",
    ]
    last_error = None
    hit_node_cap = False
    for idx_name in location_indexes:
        try:
            print(f"  osmium node-location index: {idx_name}")
            handler.apply_file(str(pbf_path), locations=True, idx=idx_name)
            last_error = None
            break
        except _ReachedNodeCap:
            hit_node_cap = True
            last_error = None
            break
        except Exception as exc:
            last_error = exc
            handler = _RoadGraphBuilder(subset_radius_m)
    if last_error is not None:
        raise last_error

    G = handler.G
    G.graph["crs"] = "EPSG:4326"

    print(
        f"  Raw PBF graph: {G.number_of_nodes():,} nodes, "
        f"{G.number_of_edges():,} edges  ({time.time()-t0:.0f}s)"
    )
    print(f"  Geographic subset radius: {subset_radius_m / 1000:.0f} km")
    if hit_node_cap:
        print(f"  Reached node cap at {MAX_GRAPH_NODES:,} nodes; stopping early.")
    print(f"  RAM after raw parse: {current_memory_gb():.1f} GB")

    G = _simplify_benchmark_graph(G)
    G = _project_graph_nodes(G)

    print(
        f"  Processed graph ready: {G.number_of_nodes():,} nodes, "
        f"{G.number_of_edges():,} edges"
    )
    print(f"  Ready.  Memory: {current_memory_gb():.1f} GB")
    return G


def load_us_graph(pbf_path: str):
    """
    Load a benchmark-ready US road graph from a PBF.

    The loader avoids the huge XML conversion step entirely:
      1. Stream the PBF directly with osmium.
      2. Keep only routing attributes used elsewhere in this script.
      3. Simplify the graph and project node coordinates once.
      4. Cache the processed NetworkX graph as a pickle for fast reruns.
    """
    pbf_path = Path(pbf_path)
    cache_path = _graph_cache_path(pbf_path)

    if cache_path.exists() and cache_path.stat().st_mtime >= pbf_path.stat().st_mtime:
        print(f"Reusing cached processed graph: {cache_path}")
        t0 = time.time()
        G = _load_cached_graph(cache_path)
        print(f"  Cache load time: {time.time()-t0:.1f}s")
        print(f"  Ready.  Memory: {current_memory_gb():.1f} GB")
        return G

    G = _build_graph_from_pbf(pbf_path)
    _save_cached_graph(G, cache_path)
    return G


# ═══════════════════════════════════════════════════════════════════════════════
# SUBGRAPH GENERATION
# ═══════════════════════════════════════════════════════════════════════════════

def _centroid_node(G):
    """Return the node nearest the geographic centre of the graph."""
    x_total = 0.0
    y_total = 0.0
    count = 0
    for _, data in G.nodes(data=True):
        x_total += data["x"]
        y_total += data["y"]
        count += 1
    cx = x_total / count
    cy = y_total / count
    return ox.distance.nearest_nodes(G, cx, cy)


def _region_name(G, nodes) -> str:
    """
    Derive a human-readable region name from node coordinates.
    Uses the bounding box centroid to pick the nearest US state name
    from a small lookup table of state centroids.
    """
    STATE_CENTROIDS = {
        "Alabama":        (32.8, -86.8),  "Alaska":         (64.2, -153.4),
        "Arizona":        (34.3, -111.1), "Arkansas":       (34.9, -92.4),
        "California":     (36.8, -119.4), "Colorado":       (39.0, -105.5),
        "Connecticut":    (41.6, -72.7),  "Delaware":       (39.0, -75.5),
        "Florida":        (28.7, -82.4),  "Georgia":        (32.7, -83.4),
        "Hawaii":         (20.9, -156.9), "Idaho":          (44.4, -114.5),
        "Illinois":       (40.0, -89.2),  "Indiana":        (39.9, -86.3),
        "Iowa":           (42.1, -93.5),  "Kansas":         (38.5, -98.4),
        "Kentucky":       (37.5, -85.3),  "Louisiana":      (31.2, -92.1),
        "Maine":          (45.4, -69.4),  "Maryland":       (39.0, -76.8),
        "Massachusetts":  (42.3, -71.8),  "Michigan":       (44.3, -85.4),
        "Minnesota":      (46.4, -93.1),  "Mississippi":    (32.7, -89.7),
        "Missouri":       (38.5, -92.5),  "Montana":        (47.0, -110.5),
        "Nebraska":       (41.5, -99.9),  "Nevada":         (39.3, -116.6),
        "New Hampshire":  (43.7, -71.6),  "New Jersey":     (40.1, -74.5),
        "New Mexico":     (34.5, -106.2), "New York":       (43.0, -75.5),
        "North Carolina": (35.6, -79.4),  "North Dakota":   (47.5, -100.5),
        "Ohio":           (40.4, -82.8),  "Oklahoma":       (35.6, -97.5),
        "Oregon":         (44.1, -120.5), "Pennsylvania":   (40.9, -77.8),
        "Rhode Island":   (41.7, -71.5),  "South Carolina": (33.8, -80.9),
        "South Dakota":   (44.4, -100.2), "Tennessee":      (35.9, -86.7),
        "Texas":          (31.5, -99.3),  "Utah":           (39.3, -111.1),
        "Vermont":        (44.1, -72.7),  "Virginia":       (37.8, -78.2),
        "Washington":     (47.4, -120.5), "West Virginia":  (38.9, -80.5),
        "Wisconsin":      (44.3, -89.6),  "Wyoming":        (43.0, -107.6),
    }

    node_data = [G.nodes[n] for n in nodes if n in G.nodes]
    if not node_data:
        return "Unknown Region"

    # Convert projected metres back to approximate lat/lon
    # (good enough for label purposes — not used for routing)
    xs = [d.get('lon', d.get('x', 0)) for d in node_data]
    ys = [d.get('lat', d.get('y', 0)) for d in node_data]
    cx = statistics.mean(xs)
    cy = statistics.mean(ys)

    best_state, best_dist = "Unknown", float('inf')
    for state, (slat, slon) in STATE_CENTROIDS.items():
        # Quick Euclidean in degrees — sufficient for label only
        d = math.sqrt((cy - slat) ** 2 + (cx - slon) ** 2)
        if d < best_dist:
            best_dist = d
            best_state = state
    return best_state


def build_subgraphs(G, n=250):
    """
    Yield (name, subgraph, origin_node, dest_node) tuples one at a time.
    Subgraphs are ego-graph expansions from the centroid node with
    linearly increasing radii so node counts span min→full.

    For very large subgraphs a random cross-country O/D pair is used.
    """
    total_nodes = G.number_of_nodes()
    centre      = _centroid_node(G)

    # Node count targets: linear interpolation from MIN_NODES → total_nodes
    targets = [
        int(MIN_NODES + (total_nodes - MIN_NODES) * i / (n - 1))
        for i in range(n)
    ]

    # We'll build subgraphs by ego expansion.
    # To avoid loading the whole adjacency into memory for every subgraph we
    # BFS-expand until we have roughly the target node count.

    print(f"\nBuilding {n} subgraphs "
          f"(≈{MIN_NODES:,} → {total_nodes:,} nodes) …")

    x_total = 0.0
    y_total = 0.0
    node_count = 0
    for _, data in G.nodes(data=True):
        x_total += data["x"]
        y_total += data["y"]
        node_count += 1
    x_mid = x_total / node_count
    y_mid = y_total / node_count

    # Keep a bounded random sample from opposite quadrants instead of
    # duplicating all coordinates into large temporary dicts/lists.
    nw_nodes = []
    se_nodes = []
    nw_seen = 0
    se_seen = 0
    for node, data in G.nodes(data=True):
        x = data["x"]
        y = data["y"]

        if x < x_mid and y >= y_mid:
            nw_seen += 1
            if len(nw_nodes) < QUADRANT_SAMPLE_LIMIT:
                nw_nodes.append(node)
            else:
                slot = random.randrange(nw_seen)
                if slot < QUADRANT_SAMPLE_LIMIT:
                    nw_nodes[slot] = node

        if x >= x_mid and y < y_mid:
            se_seen += 1
            if len(se_nodes) < QUADRANT_SAMPLE_LIMIT:
                se_nodes.append(node)
            else:
                slot = random.randrange(se_seen)
                if slot < QUADRANT_SAMPLE_LIMIT:
                    se_nodes[slot] = node

    for idx, target in enumerate(targets):
        print(f"  Subgraph {idx+1}/{n}  target={target:,} nodes …", end='\r')

        # BFS from centre until we hit the target count
        visited = {centre}
        queue   = [centre]
        while len(visited) < target and queue:
            next_q = []
            for node in queue:
                for nb in G.successors(node):
                    if nb not in visited:
                        visited.add(nb)
                        next_q.append(nb)
                        if len(visited) >= target:
                            break
                if len(visited) >= target:
                    break
            queue = next_q

        sg = G.subgraph(visited).copy()

        # Ensure the subgraph is strongly connected so all algorithms can run
        largest_scc = max(nx.strongly_connected_components(sg), key=len)
        sg = sg.subgraph(largest_scc).copy()

        sg_nodes = list(sg.nodes())
        n_sg     = len(sg_nodes)

        # Origin / destination selection
        # Small graphs: random pair within the subgraph
        # Large graphs (>50% of US): cross-country pair
        coverage = n_sg / total_nodes
        if coverage >= 0.5 and nw_nodes and se_nodes:
            # Try to pick NW and SE nodes that are in this subgraph
            nw_in = [nd for nd in nw_nodes if nd in sg]
            se_in = [nd for nd in se_nodes if nd in sg]
            if nw_in and se_in:
                origin = random.choice(nw_in)
                dest   = random.choice(se_in)
            else:
                origin, dest = random.sample(sg_nodes, 2)
        else:
            origin, dest = random.sample(sg_nodes, 2)

        # Verify path exists (SCC guarantees it but double-check)
        if not nx.has_path(sg, origin, dest):
            # Fallback: pick the two most-distant nodes in the SCC
            origin = sg_nodes[0]
            dest   = sg_nodes[-1]

        name = _region_name(G, sg_nodes)
        yield name, sg, origin, dest

    print(f"\n  Done building subgraphs.")


# ═══════════════════════════════════════════════════════════════════════════════
# GRAPH METRIC HELPERS
# ═══════════════════════════════════════════════════════════════════════════════

def graph_avg_degree(G) -> float:
    degrees = [d for _, d in G.degree()]
    return statistics.mean(degrees) if degrees else 0.0


def has_negative_cycles(G, weight=WEIGHT) -> bool:
    """True if any edge has negative weight (Bellman-Ford will flag it)."""
    for _, _, data in G.edges(data=True):
        if data.get(weight, 0) < 0:
            return True
    return False


def graph_circuitry(G) -> float:
    """
    Average circuitry = (number of edges) / (number of nodes - 1).
    A tree has circuitry = 1; more cycles push it higher.
    """
    n, m = G.number_of_nodes(), G.number_of_edges()
    if n <= 1:
        return 0.0
    return m / (n - 1)


def intersection_count(G) -> int:
    """Nodes with degree >= 3 (actual intersections)."""
    return sum(1 for _, d in G.degree() if d >= 3)


def path_travel_time(G, path, weight=WEIGHT) -> float:
    """Sum edge weights along path (handles parallel edges)."""
    total = 0.0
    for u, v in zip(path[:-1], path[1:]):
        if G.has_edge(u, v):
            costs = [d.get(weight, math.inf) for d in G[u][v].values()]
            total += min(costs)
        else:
            return math.inf
    return total


# ═══════════════════════════════════════════════════════════════════════════════
# ALGORITHM RUNNER  (with timeout + memory guard)
# ═══════════════════════════════════════════════════════════════════════════════

def run_with_limits(fn, *args, timeout=TIMEOUT_SECONDS, mem_limit_gb=MEMORY_LIMIT_GB, **kwargs):
    """
    Run fn(*args, **kwargs) with a wall-clock timeout and a memory ceiling.

    Returns:
        (result, runtime_s, mem_gb, failure_reason)
        failure_reason is None on success, or one of:
            "TIMEOUT" | "OUT OF MEMORY" | "PATH NOT FOUND" | str(exception)
    """
    result         = [None]
    exception      = [None]
    done           = threading.Event()
    mem_triggered  = [False]
    stop_monitor   = threading.Event()

    def target():
        try:
            result[0] = fn(*args, **kwargs)
        except Exception as exc:
            exception[0] = exc
        finally:
            done.set()

    monitor = threading.Thread(
        target=_memory_watcher,
        args=(stop_monitor, mem_limit_gb, mem_triggered),
        daemon=True,
    )
    worker = threading.Thread(target=target, daemon=True)

    t_start = time.perf_counter()
    monitor.start()
    worker.start()

    # Poll for completion, timeout, or memory breach
    while not done.is_set():
        elapsed = time.perf_counter() - t_start
        if elapsed >= timeout:
            stop_monitor.set()
            runtime = time.perf_counter() - t_start
            return None, runtime, current_memory_gb(), "TIMEOUT"
        if mem_triggered[0]:
            stop_monitor.set()
            runtime = time.perf_counter() - t_start
            return None, runtime, current_memory_gb(), "OUT OF MEMORY"
        time.sleep(0.25)

    stop_monitor.set()
    runtime = time.perf_counter() - t_start
    mem_used = current_memory_gb()

    if exception[0] is not None:
        return None, runtime, mem_used, f"EXCEPTION: {exception[0]}"

    return result[0], runtime, mem_used, None


# ═══════════════════════════════════════════════════════════════════════════════
# PER-ALGORITHM WRAPPERS
# ═══════════════════════════════════════════════════════════════════════════════

def run_dijkstra(G, origin, dest):
    def _run():
        return dijkstra(G, origin, dest, weight=WEIGHT)
    return run_with_limits(_run)


def run_astar(G, origin, dest):
    def _run():
        return astar(G, origin, dest, weight=WEIGHT)
    return run_with_limits(_run)


def run_bellman_ford(G, origin, dest):
    def _run():
        return bellman_ford(G, origin, dest, weight=WEIGHT)
    return run_with_limits(_run)


def run_aco(G, origin, dest):
    if G.number_of_nodes() > ACO_MAX_NODES:
        print(
            f"  [ACO] skipped — graph has {G.number_of_nodes():,} nodes "
            f"(limit {ACO_MAX_NODES:,})"
        )
        return None, 0.0, current_memory_gb(), "GRAPH SIZE TOO LARGE"

    def _run():
        # Use the parameter optimiser from myAlgorithms
        params = diagnose_aco_parameters(G, origin, dest, weight=WEIGHT)
        path, cost, _ = ant_colony_optimization(
            G, origin, dest,
            weight=WEIGHT,
            **params,
        )
        return path, cost
    return run_with_limits(_run)


# ═══════════════════════════════════════════════════════════════════════════════
# COMPARISON PLOT  (rendered in a background thread — never blocks the benchmark)
# ═══════════════════════════════════════════════════════════════════════════════

# Visual style — matches the reference image (dark background, vivid path lines)
_ALGO_STYLES = {
    "Dijkstra":    {"color": "#00FFFF", "label": "Dijkstra's"},   # cyan
    "A*":          {"color": "#FFB347", "label": "A*"},            # orange
    "Bellman-Ford":{"color": "#FF69B4", "label": "Bellman-Ford"},  # pink
    "ACO":         {"color": "#7FFF00", "label": "ACO (hybrid)"}, # chartreuse
}

# Sentinel that tells the worker thread to exit cleanly
_PLOT_POISON = None


def _extract_path(raw_result, label: str):
    """Return (path_list_or_None, cost_float, status_str) from a run result."""
    raw, runtime, mem, failure = raw_result
    if failure is not None:
        return None, math.inf, failure
    if raw is None:
        return None, math.inf, "PATH NOT FOUND"
    if isinstance(raw, tuple) and len(raw) == 3:
        path, cost, _ = raw
    else:
        path, cost = raw
    if not path or cost == math.inf:
        return None, math.inf, "PATH NOT FOUND"
    return path, cost, None


def _render_comparison(
    sg,
    origin,
    dest,
    sg_idx: int,
    name: str,
    algo_results: dict,   # {"Dijkstra": run_result, "A*": run_result, …}
    save_path: Path,
):
    """
    Draw a 4-panel comparison figure.  Each panel shows:
      • the full subgraph road network (dim grey edges)
      • the algorithm's path overlaid in its signature colour
      • origin (●) and destination (●) dots
      • a subtitle with cost (min) and runtime (s)

    Uses node x/y coordinates directly from the projected graph so no
    coordinate conversion is needed.

    This function is designed to be called from a worker thread; it creates
    and closes its own Figure so matplotlib's global state is never shared
    with the main thread.
    """
    # ── precompute node positions once ───────────────────────────────────────
    pos = {n: (d['x'], d['y']) for n, d in sg.nodes(data=True)}

    algo_keys = ["Dijkstra", "A*", "Bellman-Ford", "ACO"]
    fig, axes = plt.subplots(
        1, 4,
        figsize=(22, 6),
        facecolor="#0d0d0d",
    )
    fig.patch.set_facecolor("#0d0d0d")

    # Super-title
    fig.suptitle(
        f"Subgraph {sg_idx}  ·  {name}  ·  "
        f"{sg.number_of_nodes():,} nodes — Algorithm Comparison",
        color="white",
        fontsize=11,
        fontweight="bold",
        y=1.01,
    )

    # ── shared network edge coordinates (computed once, reused per panel) ────
    # Build arrays of line segments for all edges so we can draw them with a
    # single LineCollection call — far faster than per-edge calls on large graphs
    from matplotlib.collections import LineCollection

    edge_segments = []
    for u, v in sg.edges():
        if u in pos and v in pos:
            edge_segments.append([pos[u], pos[v]])

    for ax_idx, algo in enumerate(algo_keys):
        ax = axes[ax_idx]
        ax.set_facecolor("#0d0d0d")
        ax.set_aspect("equal")
        ax.axis("off")

        style = _ALGO_STYLES[algo]

        # ── road network background ──────────────────────────────────────────
        if edge_segments:
            lc = LineCollection(
                edge_segments,
                colors="#3a3a3a",
                linewidths=0.4,
                zorder=1,
            )
            ax.add_collection(lc)

        # ── path overlay ─────────────────────────────────────────────────────
        run_result = algo_results.get(algo)
        path, cost, failure = _extract_path(run_result, algo)
        runtime = run_result[1] if run_result else 0.0

        if path and len(path) >= 2:
            path_segs = [
                [pos[path[i]], pos[path[i + 1]]]
                for i in range(len(path) - 1)
                if path[i] in pos and path[i + 1] in pos
            ]
            if path_segs:
                # Subtle white glow behind the path for contrast
                glow = LineCollection(
                    path_segs,
                    colors="white",
                    linewidths=3.0,
                    alpha=0.15,
                    zorder=2,
                )
                ax.add_collection(glow)
                path_lc = LineCollection(
                    path_segs,
                    colors=style["color"],
                    linewidths=1.8,
                    zorder=3,
                )
                ax.add_collection(path_lc)

            # Origin and destination markers
            if origin in pos:
                ax.plot(*pos[origin], "o",
                        color=style["color"], markersize=6, zorder=5,
                        markeredgecolor="white", markeredgewidth=0.6)
            if dest in pos:
                ax.plot(*pos[dest], "o",
                        color=style["color"], markersize=6, zorder=5,
                        markeredgecolor="white", markeredgewidth=0.6)

            cost_str = f"{cost / 60:.2f} min"
            time_str = f"{runtime:.4f}s"
            subtitle  = f"{cost_str}  |  {time_str}"
        else:
            reason = failure if failure else "PATH NOT FOUND"
            subtitle = f"No path  ({reason})"

        # ── auto-scale axes to the network bounding box ──────────────────────
        if pos:
            all_x = [v[0] for v in pos.values()]
            all_y = [v[1] for v in pos.values()]
            pad_x = (max(all_x) - min(all_x)) * 0.03 or 1
            pad_y = (max(all_y) - min(all_y)) * 0.03 or 1
            ax.set_xlim(min(all_x) - pad_x, max(all_x) + pad_x)
            ax.set_ylim(min(all_y) - pad_y, max(all_y) + pad_y)

        ax.set_title(
            f"{style['label']}\n{subtitle}",
            color="white",
            fontsize=8.5,
            pad=4,
        )

    plt.tight_layout(rect=[0, 0, 1, 0.97])

    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(
        save_path,
        dpi=130,
        bbox_inches="tight",
        facecolor=fig.get_facecolor(),
    )
    plt.close(fig)


# ── Background plot queue ─────────────────────────────────────────────────────

class _PlotWorker:
    """
    Owns a single daemon thread that consumes (args, kwargs) tuples from a
    queue and calls _render_comparison() for each one.

    The benchmark loop calls  .submit(...)  which returns immediately,
    so matplotlib never stalls the algorithm timing.

    Call  .shutdown()  after the benchmark loop finishes to let all queued
    plots complete before the process exits.
    """

    def __init__(self):
        self._q      = queue.Queue()
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def _worker(self):
        while True:
            item = self._q.get()
            if item is _PLOT_POISON:
                self._q.task_done()
                break
            args, kwargs = item
            try:
                _render_comparison(*args, **kwargs)
            except Exception as exc:
                print(f"  [PlotWorker] WARNING — plot failed: {exc}")
            finally:
                self._q.task_done()

    def submit(self, *args, **kwargs):
        """Queue a plot job.  Returns immediately."""
        self._q.put((args, kwargs))

    def shutdown(self, wait=True):
        """Signal the worker to stop.  If wait=True, blocks until queue drains."""
        self._q.put(_PLOT_POISON)
        if wait:
            self._thread.join()


# ═══════════════════════════════════════════════════════════════════════════════
# CSV WRITING
# ═══════════════════════════════════════════════════════════════════════════════

HEADERS = [
    # Graph identity & topology
    "Graph_Name",
    "Num_Nodes",
    "Num_Edges",
    "Avg_Node_Degree",
    "Has_Negative_Cycles",
    "Graph_Circuitry_Avg",
    "Intersection_Count",
    "",                         # spacer column

    # ── Dijkstra ──────────────────────────────────────────────
    "Algorithm",                # always "Dijkstra"
    "Path_Found",
    "Cost_Length",
    "Path_Travel_Time",
    "Path_Node_Count",
    "Runtime_s",
    "Memory_GB",
    "Gap",                      # always "Baseline"

    # ── A* ────────────────────────────────────────────────────
    "Algorithm",
    "Path_Found",
    "Cost_Length",
    "Path_Travel_Time",
    "Path_Node_Count",
    "Runtime_s",
    "Memory_GB",
    "Gap_%_vs_Dijkstra",

    # ── Bellman-Ford ──────────────────────────────────────────
    "Algorithm",
    "Path_Found",
    "Cost_Length",
    "Path_Travel_Time",
    "Path_Node_Count",
    "Runtime_s",
    "Memory_GB",
    "Gap_%_vs_Dijkstra",

    # ── ACO ───────────────────────────────────────────────────
    "Algorithm",
    "Path_Found",
    "Cost_Length",
    "Path_Travel_Time",
    "Path_Node_Count",
    "Runtime_s",
    "Memory_GB",
    "Gap_%_vs_Dijkstra",
]


def algo_cells(label: str,
               raw_result,
               runtime: float,
               mem_gb: float,
               failure: str | None,
               G,
               dijkstra_cost: float | None) -> list:
    """
    Build the 8 CSV cells for one algorithm:
      label | path_found | cost | travel_time | node_count | runtime | mem | gap
    """
    if failure is not None:
        reason = failure  # "TIMEOUT" | "OUT OF MEMORY" | etc.
        found  = f"Path Not Found: {reason}"
        return [label, found, "", "", "", "", "", ""]

    # Unpack result (Bellman-Ford returns 3-tuple; others return 2-tuple)
    if raw_result is None:
        found = "Path Not Found: PATH NOT FOUND"
        return [label, found, "", "", "", "", "", ""]

    if isinstance(raw_result, tuple) and len(raw_result) == 3:
        path, cost, _neg_cycle = raw_result
    else:
        path, cost = raw_result

    if not path or cost == math.inf:
        found = "Path Not Found: PATH NOT FOUND"
        return [label, found, "", "", "", "", "", ""]

    travel = path_travel_time(G, path, WEIGHT)
    node_c = len(path)

    # Gap vs Dijkstra
    if dijkstra_cost and dijkstra_cost > 0 and cost < math.inf:
        gap = f"{((cost - dijkstra_cost) / dijkstra_cost) * 100:.2f}%"
    elif label == "Dijkstra":
        gap = "Baseline"
    else:
        gap = ""

    return [
        label,
        "Yes",
        f"{cost:.4f}",
        f"{travel:.4f}",
        node_c,
        f"{runtime:.3f}",
        f"{mem_gb:.3f}",
        gap,
    ]


def write_row(writer, graph_meta: dict, algo_rows: list[list]):
    """Flatten graph meta + 4 algorithm blocks into a single CSV row."""
    row = [
        graph_meta["name"],
        graph_meta["nodes"],
        graph_meta["edges"],
        f"{graph_meta['avg_degree']:.3f}",
        graph_meta["neg_cycles"],
        f"{graph_meta['circuitry']:.4f}",
        graph_meta["intersections"],
        "",                         # spacer
    ]
    for block in algo_rows:
        row.extend(block)
    writer.writerow(row)


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN BENCHMARK LOOP
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    PLOT_DIR.mkdir(parents=True, exist_ok=True)

    # ── Start background plot worker ─────────────────────────────────────────
    plot_worker = _PlotWorker()
    print(f"  Plot output → {PLOT_DIR}")

    # ── Load graph ───────────────────────────────────────────────────────────
    if not Path(PBF_PATH).exists():
        print(f"\n[ERROR] '{PBF_PATH}' not found.\n"
              "  Please download and filter the US OSM data as described\n"
              "  in the script header and place it in the same directory.\n")
        plot_worker.shutdown(wait=False)
        sys.exit(1)

    G_full = load_us_graph(PBF_PATH)

    # ── Open CSV ─────────────────────────────────────────────────────────────
    with open(OUTPUT_CSV, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(HEADERS)

        for sg_idx, (name, sg, origin, dest) in enumerate(
            build_subgraphs(G_full, n=NUM_SUBGRAPHS)
        ):
            sg_num = sg_idx + 1
            print(f"\n{'='*70}")
            print(f"Subgraph {sg_num}/{NUM_SUBGRAPHS}  |  {name}  |  "
                  f"{sg.number_of_nodes():,} nodes  |  "
                  f"{sg.number_of_edges():,} edges")
            print(f"  Origin={origin}  →  Dest={dest}")
            print(f"  RAM before: {current_memory_gb():.2f} GB")

            graph_meta = {
                "name":          name,
                "nodes":         sg.number_of_nodes(),
                "edges":         sg.number_of_edges(),
                "avg_degree":    graph_avg_degree(sg),
                "neg_cycles":    has_negative_cycles(sg),
                "circuitry":     graph_circuitry(sg),
                "intersections": intersection_count(sg),
            }

            # ── Dijkstra ─────────────────────────────────────────────────────
            print("\n  [Dijkstra] running …")
            d_run = run_dijkstra(sg, origin, dest)
            d_result, d_runtime, d_mem, d_fail = d_run
            dijk_cost = None
            if d_fail is None and d_result and d_result[1] < math.inf:
                dijk_cost = d_result[1]
            d_cells = algo_cells("Dijkstra", d_result, d_runtime, d_mem, d_fail, sg, dijk_cost)
            print(f"    → {d_cells[1]}  cost={d_cells[2]}  time={d_runtime:.2f}s")

            # ── A* ───────────────────────────────────────────────────────────
            print("\n  [A*] running …")
            a_run = run_astar(sg, origin, dest)
            a_result, a_runtime, a_mem, a_fail = a_run
            a_cells = algo_cells("A*", a_result, a_runtime, a_mem, a_fail, sg, dijk_cost)
            print(f"    → {a_cells[1]}  cost={a_cells[2]}  time={a_runtime:.2f}s")

            # ── Bellman-Ford ─────────────────────────────────────────────────
            print("\n  [Bellman-Ford] running …")
            bf_run = run_bellman_ford(sg, origin, dest)
            bf_result, bf_runtime, bf_mem, bf_fail = bf_run
            bf_cells = algo_cells("Bellman-Ford", bf_result, bf_runtime, bf_mem, bf_fail, sg, dijk_cost)
            print(f"    → {bf_cells[1]}  cost={bf_cells[2]}  time={bf_runtime:.2f}s")

            # ── ACO ──────────────────────────────────────────────────────────
            print("\n  [ACO] running …")
            ac_run = run_aco(sg, origin, dest)
            ac_result, ac_runtime, ac_mem, ac_fail = ac_run
            ac_cells = algo_cells("ACO", ac_result, ac_runtime, ac_mem, ac_fail, sg, dijk_cost)
            print(f"    → {ac_cells[1]}  cost={ac_cells[2]}  time={ac_runtime:.2f}s")

            # ── Write CSV row ─────────────────────────────────────────────────
            write_row(writer, graph_meta, [d_cells, a_cells, bf_cells, ac_cells])
            fh.flush()

            # ── Queue comparison plot (non-blocking) ──────────────────────────
            # File name: graph_001_Kansas_1000n.png  (zero-padded, sanitised)
            safe_name = name.replace(" ", "_").replace("/", "-")
            plot_fname = (
                f"graph_{sg_num:03d}_{safe_name}_{sg.number_of_nodes()}n.png"
            )
            plot_worker.submit(
                sg,
                origin,
                dest,
                sg_num,
                name,
                {
                    "Dijkstra":     d_run,
                    "A*":           a_run,
                    "Bellman-Ford": bf_run,
                    "ACO":          ac_run,
                },
                PLOT_DIR / plot_fname,
            )
            print(f"  [Plot] queued → {plot_fname}")

            # ── Clean up subgraph memory ──────────────────────────────────────
            del sg
            gc.collect()
            print(f"  RAM after GC: {current_memory_gb():.2f} GB")

    # ── Wait for all plots to finish writing ──────────────────────────────────
    print("\nBenchmark complete — waiting for remaining plots to save …")
    plot_worker.shutdown(wait=True)
    print(f"All plots saved to: {PLOT_DIR}")
    print(f"Results CSV saved to: {OUTPUT_CSV}")


if __name__ == "__main__":
    main()
