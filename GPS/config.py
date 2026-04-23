import os

# GPS serial settings
GPS_PORT = "/dev/ttyAMA0"
GPS_BAUD = 9600
GPS_TIMEOUT = 1

# Cache / graph storage
CACHE_DIR = os.path.expanduser("~/.pigps_cache")
GRAPH_CACHE = os.path.join(CACHE_DIR, "graph_cache.graphml")
NETWORK_TYPE = "drive"

# SQLite graph database
GRAPH_DB = "/home/gps_pi/Network-Routing-Optimization/GraphML_Archive/leb_cook_final.db"

# Display graph settings
GRAPH_RADIUS_M = 1500
SUBGRAPH_RADIUS_M = 2000
DISPLAY_REFRESH_MIN_MOVE_M = 150

# Route graph settings
LOCAL_ROUTE_MAX_M = 20000
LOCAL_ROUTE_RADIUS_M = 12000
LONG_ROUTE_BASE_MARGIN_M = 12000
LONG_ROUTE_MARGIN_STEPS_M = [12000, 18000, 26000, 36000]

# Algorithms
ALGORITHMS = ["A*", "Dijkstra", "Bellman-Ford", "ACO"]
DEFAULT_ALGO = "A*"
LONG_DISTANCE_ALGOS = {"A*", "Dijkstra"}
LOCAL_ONLY_ALGOS = {"Bellman-Ford", "ACO"}

ACO_ANTS = 24
ACO_ITERATIONS = 30
ACO_ALPHA = 1.0
ACO_BETA = 2.0
ACO_EVAP = 0.5

# UI
WINDOW_TITLE = "PiGPS Navigator"
WINDOW_W = 1024
WINDOW_H = 600
MAP_PANEL_W = 750
SIDEBAR_W = WINDOW_W - MAP_PANEL_W

# Colors
BG_DARK = "#0d1117"
BG_MID = "#161b22"
BG_LIGHT = "#21262d"
ACCENT = "#39d353"
ACCENT2 = "#58a6ff"
TEXT_PRIMARY = "#e6edf3"
TEXT_MUTED = "#8b949e"
NODE_CLR = "#30363d"
EDGE_CLR = "#21262d"
ROUTE_CLR = "#58a6ff"
POSITION_CLR = "#39d353"
DEST_CLR = "#f85149"

# Runtime
GPS_POLL_MS = 1000
MAP_REFRESH_MS = 1000
REROUTE_DIST_M = 50
OFF_ROUTE_THRESHOLD_M = 120
REROUTE_COOLDOWN_S = 15
