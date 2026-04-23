# config.py — Central configuration for PiGPS Navigator

import os

# ── GPS Serial Settings ──────────────────────────────────────────────────────
GPS_PORT        = "/dev/ttyAMA0"   # Pi Zero 2W UART; change to /dev/ttyAMA0 if needed
GPS_BAUD        = 9600
GPS_TIMEOUT     = 1              # seconds

# ── OSMnx / Map Settings ─────────────────────────────────────────────────────
CACHE_DIR       = os.path.expanduser("~/.pigps_cache")
GRAPH_CACHE     = os.path.join(CACHE_DIR, "graph_cache.graphml")
GRAPH_RADIUS_M  = 1000           # metres around current location to download
NETWORK_TYPE    = "drive"        # 'drive' | 'walk' | 'bike'

# ── Pre-downloaded GraphML ────────────────────────────────────────────────────
# Set this to the path of your pre-downloaded GraphML file.
# The app will load this instead of downloading from OSM.
GRAPH_DB = "/home/gps_pi/Network-Routing-Optimization/GraphML_Archive/leb_cook_final.db"

# Subgraph viewport settings
SUBGRAPH_RADIUS_M = 2000   # metres around user to extract for rendering
ROUTING_RADIUS_M  = 50000   # metres around user to extract for routing


# ── UI Settings ──────────────────────────────────────────────────────────────
WINDOW_TITLE    = "PiGPS Navigator"
WINDOW_W        = 1024
WINDOW_H        = 600
MAP_PANEL_W     = 750            # left canvas width
SIDEBAR_W       = WINDOW_W - MAP_PANEL_W

# ── Colours ──────────────────────────────────────────────────────────────────
BG_DARK         = "#0d1117"
BG_MID          = "#161b22"
BG_LIGHT        = "#21262d"
ACCENT          = "#39d353"      # GPS green
ACCENT2         = "#58a6ff"      # route blue
TEXT_PRIMARY    = "#e6edf3"
TEXT_MUTED      = "#8b949e"
NODE_CLR        = "#30363d"
EDGE_CLR        = "#21262d"
ROUTE_CLR       = "#58a6ff"
POSITION_CLR    = "#39d353"
DEST_CLR        = "#f85149"

# ── Routing ──────────────────────────────────────────────────────────────────
ALGORITHMS = ["A*", "Dijkstra", "Bellman-Ford", "ACO", "Bi-Dijkstra", "Bi-A*"]
DEFAULT_ALGO    = "A*"
ACO_ANTS        = 30
ACO_ITERATIONS  = 50
ACO_ALPHA       = 1.0            # pheromone importance
ACO_BETA        = 2.0            # heuristic importance
ACO_EVAP        = 0.5            # evaporation rate

# ── Misc ─────────────────────────────────────────────────────────────────────
GPS_POLL_MS     = 1000           # how often to poll GPS (milliseconds)
MAP_REFRESH_MS  = 1000           # how often to redraw the map
REROUTE_DIST_M  = 50             # metres off-route before re-routing
