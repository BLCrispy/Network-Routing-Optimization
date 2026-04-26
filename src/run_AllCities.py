import matplotlib
matplotlib.use("Agg")

import os
import csv
import time
import math
import tracemalloc

import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt

import myAlgorithms as ma


# ============================================================
# SETTINGS
# ============================================================

TARGET_TRAVEL_TIME_SECONDS = 20 * 60
GRAPH_FOLDER = "GraphML_Archive"
RESULTS_CSV = "city_network_experiment_results.csv"
IMAGE_FOLDER = "city_route_images"

RUN_ACO = True


# ============================================================
# CITY LIST
# ============================================================

cities = [
    "Cookeville, Tennessee, USA",
    "Lebanon, Tennessee, USA",
    "Murfreesboro, Tennessee, USA",
    "Chattanooga, Tennessee, USA",
    "Knoxville, Tennessee, USA",
    "Memphis, Tennessee, USA",
    "Nashville, Tennessee, USA",
    "Johnson City, Tennessee, USA",
    "Jackson, Tennessee, USA",
    "Clarksville, Tennessee, USA",

    "Charlotte, North Carolina, USA",
    "Raleigh, North Carolina, USA",
    "Greensboro, North Carolina, USA",
    "Asheville, North Carolina, USA",

    "Atlanta, Georgia, USA",
    "Savannah, Georgia, USA",
    "Birmingham, Alabama, USA",
    "Huntsville, Alabama, USA",
    "Louisville, Kentucky, USA",
    "Lexington, Kentucky, USA",
]


# ============================================================
# PATH SETUP
# ============================================================

current_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.dirname(current_dir)

graph_dir = os.path.join(repo_root, GRAPH_FOLDER)
image_dir = os.path.join(current_dir, IMAGE_FOLDER)

os.makedirs(graph_dir, exist_ok=True)
os.makedirs(image_dir, exist_ok=True)


# ============================================================
# HELPERS
# ============================================================

def safe_filename(name):
    bad_chars = ['<', '>', ':', '"', '/', '\\', '|', '?', '*']
    for ch in bad_chars:
        name = name.replace(ch, "_")
    return name.replace(", ", "_").replace(" ", "_")


def load_or_download_graph(place_name):
    filename = safe_filename(place_name) + ".graphml"
    graph_path = os.path.join(graph_dir, filename)

    if os.path.exists(graph_path):
        print(f"Loading saved graph: {filename}")
        G = ox.load_graphml(graph_path)
    else:
        print(f"Downloading graph for: {place_name}")
        G = ox.graph_from_place(place_name, network_type="drive")
        ox.save_graphml(G, filepath=graph_path)
        print(f"Saved graph: {filename}")

    G = ox.add_edge_speeds(G)
    G = ox.add_edge_travel_times(G)

    return G


def get_center_node(G):
    xs = [data["x"] for _, data in G.nodes(data=True)]
    ys = [data["y"] for _, data in G.nodes(data=True)]

    center_x = sum(xs) / len(xs)
    center_y = sum(ys) / len(ys)

    return ox.nearest_nodes(G, X=center_x, Y=center_y)


def get_node_coords(G, node):
    return G.nodes[node]["y"], G.nodes[node]["x"]


def choose_destination_around_20_min(G, origin_node):
    lengths, paths = nx.single_source_dijkstra(
        G,
        origin_node,
        cutoff=TARGET_TRAVEL_TIME_SECONDS * 1.75,
        weight="travel_time"
    )

    possible_nodes = [
        node for node, cost in lengths.items()
        if cost >= TARGET_TRAVEL_TIME_SECONDS * 0.5
    ]

    if not possible_nodes:
        possible_nodes = list(lengths.keys())

    dest_node = min(
        possible_nodes,
        key=lambda node: abs(lengths[node] - TARGET_TRAVEL_TIME_SECONDS)
    )

    return dest_node, paths[dest_node], lengths[dest_node]


def path_length_miles(G, path):
    total = 0

    for u, v in zip(path[:-1], path[1:]):
        edge_data = min(
            G[u][v].values(),
            key=lambda e: e.get("length", float("inf"))
        )
        total += edge_data.get("length", 0)

    return total / 1609.34


def intersection_count(G):
    count = 0

    for node in G.nodes():
        degree = len(set(G.predecessors(node)).union(set(G.successors(node))))
        if degree >= 3:
            count += 1

    return count


def avg_degree(G):
    if G.number_of_nodes() == 0:
        return 0
    return sum(dict(G.degree()).values()) / G.number_of_nodes()


def avg_circuity(G):
    circuities = []

    for u, v, data in G.edges(data=True):
        length = data.get("length", None)

        if not length or length <= 0:
            continue

        x1, y1 = G.nodes[u]["x"], G.nodes[u]["y"]
        x2, y2 = G.nodes[v]["x"], G.nodes[v]["y"]

        straight = ox.distance.great_circle(y1, x1, y2, x2)

        if straight and straight > 0:
            circuities.append(length / straight)

    if not circuities:
        return ""

    return sum(circuities) / len(circuities)


def run_with_memory(func):
    tracemalloc.start()
    t0 = time.time()

    try:
        result = func()
        error = ""
    except Exception as e:
        result = None
        error = str(e)

    runtime = time.time() - t0
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    memory_mb = peak / (1024 * 1024)

    return result, runtime, memory_mb, error


def save_route_image(
    G_projected,
    route_name,
    algorithm_name,
    path,
    runtime,
    memory_mb,
    distance_miles,
    travel_time_min,
    origin_lat,
    origin_lon,
    dest_lat,
    dest_lon
):
    if not path:
        return ""

    filename = f"{safe_filename(route_name)}_{safe_filename(algorithm_name)}.png"
    image_path = os.path.join(image_dir, filename)

    color = {
        "Dijkstra": "cyan",
        "A*": "orange",
        "Bellman-Ford": "magenta",
        "ACO": "lime",
    }.get(algorithm_name, "red")

    fig, ax = ox.plot_graph_route(
        G_projected,
        path,
        route_color=color,
        route_linewidth=4,
        node_size=0,
        edge_color="black",
        edge_linewidth=0.6,
        bgcolor="white",
        show=False,
        close=False
    )

    title = (
        f"{algorithm_name} | Runtime: {runtime:.3f}s | Memory: {memory_mb:.1f} MB | "
        f"Distance: {distance_miles:.2f} mi | Travel Time: {travel_time_min:.2f} min\n"
        f"Start: ({origin_lat:.5f}, {origin_lon:.5f}) → "
        f"End: ({dest_lat:.5f}, {dest_lon:.5f})"
    )

    ax.set_title(title, fontsize=10, fontweight="bold")

    plt.savefig(image_path, dpi=150, bbox_inches="tight")
    plt.close(fig)

    return image_path


# ============================================================
# MAIN EXPERIMENT
# ============================================================

rows = []

for city in cities:
    print("\n" + "=" * 70)
    print(f"STARTING CITY: {city}")
    print("=" * 70)

    try:
        G = load_or_download_graph(city)

        print("Projecting graph...")
        G_projected = ox.project_graph(G)

        graph_nodes_n = G.number_of_nodes()
        graph_edges_m = G.number_of_edges()
        k_avg = avg_degree(G)
        intersections = intersection_count(G)
        circuity = avg_circuity(G)

        origin_node = get_center_node(G)
        dest_node, reference_path, reference_time = choose_destination_around_20_min(
            G,
            origin_node
        )

        origin_lat, origin_lon = get_node_coords(G, origin_node)
        dest_lat, dest_lon = get_node_coords(G, dest_node)

        route_name = city.split(",")[0]

        print(f"Reference route time: {reference_time / 60:.2f} min")
        print(f"Origin: ({origin_lat}, {origin_lon})")
        print(f"Destination: ({dest_lat}, {dest_lon})")

        algorithms = [
            ("Dijkstra", lambda: ma.dijkstra(
                G_projected,
                origin_node,
                dest_node,
                weight="travel_time"
            )),
            ("A*", lambda: ma.astar(
                G_projected,
                origin_node,
                dest_node,
                weight="travel_time"
            )),
            ("Bellman-Ford", lambda: ma.bellman_ford(
                G_projected,
                origin_node,
                dest_node,
                weight="travel_time"
            )),
        ]

        if RUN_ACO:
            algorithms.append(
                ("ACO", lambda: ma.ant_colony_optimization(
                    G_projected,
                    origin_node,
                    dest_node,
                    n_ants=40,
                    n_iterations=100,
                    alpha=1.0,
                    beta=5.0,
                    evaporation_rate=0.15,
                    deposit_weight=1000,
                    weight="travel_time",
                    padding=5000,
                    max_steps=1500
                ))
            )

        for algorithm_name, algorithm_func in algorithms:
            print(f"Running {algorithm_name}...")

            result, runtime, memory_mb, error = run_with_memory(algorithm_func)

            has_neg_cycle = ""

            if error:
                path = []
                cost = math.inf

            elif algorithm_name == "Bellman-Ford":
                path, cost, has_neg_cycle = result

            elif algorithm_name == "ACO":
                path, cost, history = result

            else:
                path, cost = result

            found_path = bool(path) and cost < math.inf

            if found_path:
                travel_time_min = cost / 60
                distance_miles = path_length_miles(G_projected, path)

                save_route_image(
                    G_projected,
                    route_name,
                    algorithm_name,
                    path,
                    runtime,
                    memory_mb,
                    distance_miles,
                    travel_time_min,
                    origin_lat,
                    origin_lon,
                    dest_lat,
                    dest_lon
                )
            else:
                travel_time_min = "No path"
                distance_miles = "No path"

            row = {
                "City": city,
                "Algorithm": algorithm_name,

                "Origin Latitude": origin_lat,
                "Origin Longitude": origin_lon,
                "Destination Latitude": dest_lat,
                "Destination Longitude": dest_lon,

                "Found Path": found_path,
                "Travel Time Min": travel_time_min,
                "Runtime Seconds": runtime,
                "Memory MB": memory_mb,
                "Path Nodes": len(path),

                "Graph Nodes n": graph_nodes_n,
                "Graph Edges m": graph_edges_m,
                "Average Degree k_avg": k_avg,
                "Intersection Count": intersections,
                "Circuity Avg": circuity,

                "Reference Route Time Min": reference_time / 60,
                "Route Distance Miles": distance_miles,
                "Negative Cycle": has_neg_cycle,
                "Error": error,
            }

            rows.append(row)

            print(
                f"{algorithm_name}: "
                f"runtime={runtime:.4f}s | "
                f"memory={memory_mb:.2f}MB | "
                f"found={found_path}"
            )

    except Exception as e:
        print(f"FAILED CITY {city}: {e}")

        rows.append({
            "City": city,
            "Algorithm": "CITY FAILED",

            "Origin Latitude": "",
            "Origin Longitude": "",
            "Destination Latitude": "",
            "Destination Longitude": "",

            "Found Path": False,
            "Travel Time Min": "",
            "Runtime Seconds": "",
            "Memory MB": "",
            "Path Nodes": "",

            "Graph Nodes n": "",
            "Graph Edges m": "",
            "Average Degree k_avg": "",
            "Intersection Count": "",
            "Circuity Avg": "",

            "Reference Route Time Min": "",
            "Route Distance Miles": "",
            "Negative Cycle": "",
            "Error": str(e),
        })


# ============================================================
# SAVE CSV
# ============================================================

if rows:
    csv_path = os.path.join(current_dir, RESULTS_CSV)

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    print("\n" + "=" * 70)
    print("DONE")
    print(f"Saved city network experiment CSV to: {csv_path}")
    print(f"Saved route images to: {image_dir}")
    print("=" * 70)

else:
    print("No results saved.")
