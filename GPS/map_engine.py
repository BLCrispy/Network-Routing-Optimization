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
    ACCENT, ACCENT2, TEXT_MUTED, PRELOADED_GRAPH
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
        self.G: Optional[nx.MultiDiGraph] = None
        self._lock    = threading.Lock()
        self._center  = (None, None)

        if OSMNX_OK:
            ox.settings.use_cache    = True
            ox.settings.cache_folder = CACHE_DIR
            ox.settings.log_console  = False

    def load(self, lat: float, lon: float,
             radius_m: int = GRAPH_RADIUS_M,
             on_done=None, on_error=None):

        def _worker():
            try:
                print(f"[WORKER] PRELOADED_GRAPH = {PRELOADED_GRAPH}")
                print(f"[WORKER] exists = {os.path.exists(PRELOADED_GRAPH)}")

                if PRELOADED_GRAPH and os.path.exists(PRELOADED_GRAPH):
                    print("[WORKER] Loading from pickle...")
                    with open(PRELOADED_GRAPH, "rb") as f:
                        G = pickle.load(f)
                    print(f"[WORKER] Pickle loaded: {G.number_of_nodes()} nodes")
                else:
                    print("[WORKER] No preloaded file, downloading...")
                    if not OSMNX_OK:
                        raise RuntimeError("OSMnx not installed")
                    G = ox.graph_from_point(
                        (lat, lon),
                        dist=radius_m,
                        network_type=NETWORK_TYPE,
                        simplify=True,
                    )

                with self._lock:
                    self.G = G
                    self._center = (lat, lon)

                logger.info(f"Graph loaded: {G.number_of_nodes()} nodes, {G.number_of_edges()} edges")
                if on_done:
                    on_done(G)

            except Exception as e:
                import traceback
                traceback.print_exc()
                logger.error(f"Graph load failed: {e}")
                if on_error:
                    on_error(str(e))

        print("[LOADER] Spawning worker thread...")
        t = threading.Thread(target=_worker, daemon=True)
        t.start()
        print(f"[LOADER] Thread started: {t.name}")

    def nearest_node(self, lat: float, lon: float) -> Optional[int]:
        with self._lock:
            if self.G is None or not OSMNX_OK:
                return None
            return ox.nearest_nodes(self.G, lon, lat)

    def geocode(self, address: str) -> Optional[Tuple[float, float]]:
        if not OSMNX_OK:
            return None
        try:
            return ox.geocode(address)
        except Exception as e:
            logger.error(f"Geocode failed for '{address}': {e}")
            return None


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