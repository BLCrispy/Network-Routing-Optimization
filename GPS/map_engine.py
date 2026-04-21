# map_engine.py — OSMnx graph management & Tkinter canvas renderer

import os
import math
import logging
import pickle
import threading
from typing import Optional, List, Tuple

import networkx as nx

try:
    import osmnx as ox
    OSMNX_OK = True
except ImportError:
    OSMNX_OK = False

from config import (
    CACHE_DIR, GRAPH_RADIUS_M, NETWORK_TYPE,
    NODE_CLR, EDGE_CLR, ROUTE_CLR, POSITION_CLR, DEST_CLR,
    ACCENT, ACCENT2, TEXT_MUTED, PRELOADED_GRAPH, SUBGRAPH_RADIUS_M, ROUTING_RADIUS_M
)

logger = logging.getLogger(__name__)
os.makedirs(CACHE_DIR, exist_ok=True)


class Viewport:
    def __init__(self, canvas_w: int, canvas_h: int):
        self.w = canvas_w
        self.h = canvas_h
        self.center_lat  = 0.0
        self.center_lon  = 0.0
        self.zoom        = 1.0
        self._base_scale = 0.0

    def fit_graph(self, G: nx.MultiDiGraph):
        lats = [d["y"] for _, d in G.nodes(data=True)]
        lons = [d["x"] for _, d in G.nodes(data=True)]
        self.center_lat = (min(lats) + max(lats)) / 2
        self.center_lon = (min(lons) + max(lons)) / 2
        lat_span = (max(lats) - min(lats)) or 0.001
        lon_span = (max(lons) - min(lons)) or 0.001
        m_per_deg_lat = 111_320
        m_per_deg_lon = 111_320 * math.cos(math.radians(self.center_lat))
        height_m = lat_span * m_per_deg_lat
        width_m  = lon_span * m_per_deg_lon
        scale_h = self.h / height_m if height_m else 1
        scale_w = self.w / width_m  if width_m  else 1
        self._base_scale = min(scale_h, scale_w) * 0.85
        self.zoom = 1.0

    def geo_to_canvas(self, lat: float, lon: float) -> Tuple[float, float]:
        m_per_deg_lat = 111_320
        m_per_deg_lon = 111_320 * math.cos(math.radians(self.center_lat))
        scale = self._base_scale * self.zoom
        dx =  (lon - self.center_lon) * m_per_deg_lon * scale
        dy = -(lat - self.center_lat) * m_per_deg_lat * scale
        return self.w / 2 + dx, self.h / 2 + dy

    def canvas_to_geo(self, cx: float, cy: float) -> Tuple[float, float]:
        m_per_deg_lat = 111_320
        m_per_deg_lon = 111_320 * math.cos(math.radians(self.center_lat))
        scale = self._base_scale * self.zoom
        dx = cx - self.w / 2
        dy = cy - self.h / 2
        lon = self.center_lon + dx / (m_per_deg_lon * scale)
        lat = self.center_lat - dy / (m_per_deg_lat * scale)
        return lat, lon

    def pan(self, dlat: float, dlon: float):
        self.center_lat += dlat
        self.center_lon += dlon

    def zoom_in(self):
        self.zoom = min(self.zoom * 1.3, 20.0)

    def zoom_out(self):
        self.zoom = max(self.zoom / 1.3, 0.1)


class GraphLoader:
    def __init__(self):
        self.G: Optional[nx.MultiDiGraph] = None      # active subgraph for display
        self.G_full: Optional[nx.MultiDiGraph] = None # full city graph (read-only)
        self._lock   = threading.Lock()
        self._center = (None, None)
        self._loading = False

        if OSMNX_OK:
            ox.settings.use_cache    = True
            ox.settings.cache_folder = CACHE_DIR
            ox.settings.log_console  = False

    def load_full_graph(self, on_done=None, on_error=None):
        """Load the full city graph from pickle into memory once."""
        def _worker():
            try:
                print(f"[WORKER] Loading full graph from {PRELOADED_GRAPH}")
                with open(PRELOADED_GRAPH, "rb") as f:
                    G_full = pickle.load(f)
                print(f"[WORKER] Full graph loaded: {G_full.number_of_nodes()} nodes")
                with self._lock:
                    self.G_full = G_full
                if on_done:
                    on_done(G_full)
            except Exception as e:
                import traceback
                traceback.print_exc()
                logger.error(f"Full graph load failed: {e}")
                if on_error:
                    on_error(str(e))

        print("[LOADER] Loading full graph...")
        t = threading.Thread(target=_worker, daemon=True)
        t.start()

    def extract_subgraph(self, lat: float, lon: float,
                          radius_m: int = SUBGRAPH_RADIUS_M) -> Optional[nx.MultiDiGraph]:
        """
        Extract a lightweight subgraph around a point from the full graph.
        This is fast — pure in-memory node filtering, no download.
        """
        with self._lock:
            if self.G_full is None:
                return None
            G_full = self.G_full

        # Find all nodes within radius_m metres
        R = 6_371_000
        nodes_in_radius = []
        for node, data in G_full.nodes(data=True):
            node_lat = data["y"]
            node_lon = data["x"]
            dlat = math.radians(node_lat - lat)
            dlon = math.radians(node_lon - lon)
            a = (math.sin(dlat/2)**2 +
                 math.cos(math.radians(lat)) *
                 math.cos(math.radians(node_lat)) *
                 math.sin(dlon/2)**2)
            dist = R * 2 * math.asin(math.sqrt(a))
            if dist <= radius_m:
                nodes_in_radius.append(node)

        if not nodes_in_radius:
            return None

        subgraph = G_full.subgraph(nodes_in_radius).copy()
        return subgraph

    def update_subgraph(self, lat: float, lon: float):
        """Update the active display subgraph around current position."""
        sub = self.extract_subgraph(lat, lon, SUBGRAPH_RADIUS_M)
        if sub:
            with self._lock:
                self.G = sub
                self._center = (lat, lon)

    def get_routing_graph(self, lat: float, lon: float) -> Optional[nx.MultiDiGraph]:
        """Get a larger subgraph suitable for routing calculations."""
        return self.extract_subgraph(lat, lon, ROUTING_RADIUS_M)

    def nearest_node(self, lat: float, lon: float,
                     use_full: bool = True) -> Optional[int]:
        """Find nearest node — uses full graph for accuracy."""
        with self._lock:
            G = self.G_full if use_full else self.G
            if G is None or not OSMNX_OK:
                return None
            return ox.nearest_nodes(G, lon, lat)

    def geocode(self, address: str) -> Optional[Tuple[float, float]]:
        if not OSMNX_OK:
            return None
        try:
            return ox.geocode(address)
        except Exception as e:
            logger.error(f"Geocode failed for '{address}': {e}")
            return None

    # Keep load() as an alias so main.py doesn't break
    def load(self, lat: float, lon: float,
             radius_m: int = SUBGRAPH_RADIUS_M,
             on_done=None, on_error=None):
        self.load_full_graph(on_done=on_done, on_error=on_error)


class MapRenderer:
    def __init__(self, canvas, viewport: Viewport):
        self.canvas = canvas
        self.vp     = viewport
        self._tag   = "map_layer"

    def render(self, G, route_nodes=(), position=None, destination=None):
        c = self.canvas
        c.delete(self._tag)

        if G is None:
            c.create_text(
                self.vp.w // 2, self.vp.h // 2,
                text="Waiting for map data…",
                fill=TEXT_MUTED, font=("Courier", 14),
                tags=self._tag,
            )
            return

        self._draw_edges(G)
        self._draw_nodes(G)
        if route_nodes:
            self._draw_route(G, route_nodes)
        if destination:
            self._draw_marker(destination[0], destination[1], DEST_CLR, "✕")
        if position:
            self._draw_position(position[0], position[1])

    def _offscreen(self, x, y, margin=20):
        return (x < -margin or x > self.vp.w + margin or
                y < -margin or y > self.vp.h + margin)

    def _draw_edges(self, G):
        for u, v, _ in G.edges(data=True):
            if u not in G.nodes or v not in G.nodes:
                continue
            x1, y1 = self.vp.geo_to_canvas(G.nodes[u]["y"], G.nodes[u]["x"])
            x2, y2 = self.vp.geo_to_canvas(G.nodes[v]["y"], G.nodes[v]["x"])
            if self._offscreen(x1, y1) and self._offscreen(x2, y2):
                continue
            self.canvas.create_line(x1, y1, x2, y2,
                fill=EDGE_CLR, width=1.5, tags=self._tag)

    def _draw_nodes(self, G):
        if self.vp.zoom < 3:
            return
        for node, data in G.nodes(data=True):
            cx, cy = self.vp.geo_to_canvas(data["y"], data["x"])
            if self._offscreen(cx, cy):
                continue
            r = 2
            self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r,
                fill=NODE_CLR, outline="", tags=self._tag)

    def _draw_route(self, G, route_nodes):
        coords = []
        for node in route_nodes:
            if node not in G.nodes:
                continue
            d = G.nodes[node]
            cx, cy = self.vp.geo_to_canvas(d["y"], d["x"])
            coords.extend([cx, cy])
        if len(coords) >= 4:
            self.canvas.create_line(*coords,
                fill=ROUTE_CLR, width=4,
                capstyle="round", joinstyle="round",
                tags=self._tag)

    def _draw_position(self, lat, lon):
        cx, cy = self.vp.geo_to_canvas(lat, lon)
        for r, color in [(18, "#0d1117"), (14, "#1a6b29"), (9, ACCENT)]:
            self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r,
                fill=color, outline=ACCENT, width=2, tags=self._tag)
        self.canvas.create_text(cx, cy, text="◉",
            fill=POSITION_CLR, font=("TkDefaultFont", 12, "bold"),
            tags=self._tag)

    def _draw_marker(self, lat, lon, color, symbol):
        cx, cy = self.vp.geo_to_canvas(lat, lon)
        r = 10
        self.canvas.create_oval(cx-r, cy-r, cx+r, cy+r,
            fill=color, outline="white", width=2, tags=self._tag)
        self.canvas.create_text(cx, cy, text=symbol,
            fill="white", font=("TkDefaultFont", 9, "bold"),
            tags=self._tag)