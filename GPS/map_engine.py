import os
import math
import logging
import sqlite3
import threading
from typing import Optional, List, Tuple

import networkx as nx

try:
    import osmnx as ox
    OSMNX_OK = True
except ImportError:
    OSMNX_OK = False

from config import (
    CACHE_DIR,
    GRAPH_DB,
    SUBGRAPH_RADIUS_M,
    LOCAL_ROUTE_MAX_M,
    LOCAL_ROUTE_RADIUS_M,
    LONG_ROUTE_BASE_MARGIN_M,
    NODE_CLR,
    EDGE_CLR,
    ROUTE_CLR,
    POSITION_CLR,
    DEST_CLR,
    ACCENT,
    TEXT_MUTED,
)

logger = logging.getLogger(__name__)
os.makedirs(CACHE_DIR, exist_ok=True)


def _haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6_371_000
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = (
        math.sin(dlat / 2) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    )
    return r * 2 * math.asin(math.sqrt(a))


class Viewport:
    def __init__(self, canvas_w: int, canvas_h: int):
        self.w = canvas_w
        self.h = canvas_h
        self.center_lat = 36.18
        self.center_lon = -85.50
        self.zoom = 1.0
        self._base_scale = 100.0

    def fit_graph(self, G: nx.MultiDiGraph):
        if G is None or G.number_of_nodes() == 0:
            return

        lats = [d["y"] for _, d in G.nodes(data=True)]
        lons = [d["x"] for _, d in G.nodes(data=True)]

        self.center_lat = (min(lats) + max(lats)) / 2
        self.center_lon = (min(lons) + max(lons)) / 2

        lat_span = (max(lats) - min(lats)) or 0.001
        lon_span = (max(lons) - min(lons)) or 0.001

        m_per_deg_lat = 111_320
        m_per_deg_lon = 111_320 * math.cos(math.radians(self.center_lat))

        height_m = lat_span * m_per_deg_lat
        width_m = lon_span * m_per_deg_lon

        scale_h = self.h / height_m if height_m else 1.0
        scale_w = self.w / width_m if width_m else 1.0

        self._base_scale = min(scale_h, scale_w) * 0.85
        self.zoom = 1.6

    def geo_to_canvas(self, lat: float, lon: float) -> Tuple[float, float]:
        m_per_deg_lat = 111_320
        m_per_deg_lon = 111_320 * math.cos(math.radians(self.center_lat))
        scale = self._base_scale * self.zoom

        dx = (lon - self.center_lon) * m_per_deg_lon * scale
        dy = -(lat - self.center_lat) * m_per_deg_lat * scale
        return self.w / 2 + dx, self.h / 2 + dy

    def zoom_in(self):
        self.zoom = min(self.zoom * 1.3, 20.0)

    def zoom_out(self):
        self.zoom = max(self.zoom / 1.3, 0.1)


class GraphLoader:
    def __init__(self):
        self.G: Optional[nx.MultiDiGraph] = None
        self.G_route: Optional[nx.MultiDiGraph] = None

        self._lock = threading.Lock()
        self._db_path = GRAPH_DB
        self._db_ok = os.path.exists(self._db_path)
        self._subgraph_loading = False

        if not self._db_ok:
            logger.error("Graph DB not found: %s", self._db_path)

        if OSMNX_OK:
            ox.settings.use_cache = True
            ox.settings.cache_folder = CACHE_DIR
            ox.settings.log_console = False

    def load(self, lat: float, lon: float, radius_m: int = SUBGRAPH_RADIUS_M, on_done=None, on_error=None):
        def _worker():
            try:
                sub = self._query_radius_graph(lat, lon, radius_m)
                if sub is None or sub.number_of_nodes() == 0:
                    raise RuntimeError("No nodes found near GPS position")

                with self._lock:
                    self.G = sub

                if on_done:
                    on_done(sub)
            except Exception as e:
                logger.exception("Display graph load failed")
                if on_error:
                    on_error(str(e))

        threading.Thread(target=_worker, daemon=True).start()

    def update_subgraph(self, lat: float, lon: float, radius_m: int = SUBGRAPH_RADIUS_M):
        if self._subgraph_loading:
            return

        self._subgraph_loading = True

        def _bg():
            try:
                sub = self._query_radius_graph(lat, lon, radius_m)
                if sub is not None and sub.number_of_nodes() > 0:
                    with self._lock:
                        self.G = sub
            finally:
                self._subgraph_loading = False

        threading.Thread(target=_bg, daemon=True).start()

    def load_route_graph(
        self,
        orig_lat: float,
        orig_lon: float,
        dest_lat: float,
        dest_lon: float,
        forced_margin_m: Optional[float] = None,
    ) -> Optional[nx.MultiDiGraph]:
        if not self._db_ok:
            return None

        trip_m = _haversine(orig_lat, orig_lon, dest_lat, dest_lon)

        if trip_m <= LOCAL_ROUTE_MAX_M:
            center_lat = (orig_lat + dest_lat) / 2.0
            center_lon = (orig_lon + dest_lon) / 2.0
            radius_m = max(LOCAL_ROUTE_RADIUS_M, trip_m * 0.75 + 3000)
            G = self._query_radius_graph(center_lat, center_lon, radius_m)
        else:
            margin_m = forced_margin_m if forced_margin_m is not None else max(
                LONG_ROUTE_BASE_MARGIN_M, trip_m * 0.15
            )
            min_lat, max_lat, min_lon, max_lon = self._bbox_with_margin(
                orig_lat, orig_lon, dest_lat, dest_lon, margin_m
            )
            G = self._query_bbox_graph(min_lat, max_lat, min_lon, max_lon)

        with self._lock:
            self.G_route = G
        return G

    def nearest_node(self, lat: float, lon: float) -> Optional[int]:
        if not self._db_ok:
            return None

        search_margins = [0.01, 0.03, 0.08]

        try:
            conn = sqlite3.connect(self._db_path, timeout=15)
            cur = conn.cursor()

            for deg_margin in search_margins:
                lon_scale = 111_320 * max(math.cos(math.radians(lat)), 0.2)
                cur.execute(
                    """
                    SELECT id,
                           (((lat - ?) * 111320.0) * ((lat - ?) * 111320.0) +
                            ((lon - ?) * ?) * ((lon - ?) * ?)) AS dist2
                    FROM nodes
                    WHERE lat BETWEEN ? AND ?
                      AND lon BETWEEN ? AND ?
                    ORDER BY dist2 ASC
                    LIMIT 1
                    """,
                    (
                        lat, lat,
                        lon, lon_scale,
                        lon, lon_scale,
                        lat - deg_margin, lat + deg_margin,
                        lon - deg_margin, lon + deg_margin,
                    ),
                )
                row = cur.fetchone()
                if row:
                    conn.close()
                    return int(row[0])

            conn.close()
            return None
        except Exception as e:
            logger.error("nearest_node error: %s", e)
            return None

    def geocode(self, address: str) -> Optional[Tuple[float, float]]:
        if not OSMNX_OK:
            return None

        queries = [address]

        if "TN" not in address.upper() and "TENNESSEE" not in address.upper():
            queries.append(f"{address}, Tennessee, USA")

        parts = [p.strip() for p in address.split(",") if p.strip()]
        if len(parts) >= 2:
            queries.append(", ".join(parts[-2:]))

        queries.append(f"{address}, Tennessee")

        for query in queries:
            try:
                result = ox.geocode(query)
                if result:
                    lat, lon = result
                    if 35.5 <= lat <= 36.8 and -87.0 <= lon <= -85.0:
                        logger.info("Geocoded '%s' -> %.5f, %.5f", query, lat, lon)
                        return float(lat), float(lon)
            except Exception:
                continue

        logger.error("All geocode attempts failed for '%s'", address)
        return None

    def _bbox_with_margin(
        self,
        lat1: float,
        lon1: float,
        lat2: float,
        lon2: float,
        margin_m: float,
    ) -> Tuple[float, float, float, float]:
        center_lat = (lat1 + lat2) / 2.0
        deg_lat = margin_m / 111_320
        deg_lon = margin_m / max(111_320 * math.cos(math.radians(center_lat)), 1e-6)

        min_lat = min(lat1, lat2) - deg_lat
        max_lat = max(lat1, lat2) + deg_lat
        min_lon = min(lon1, lon2) - deg_lon
        max_lon = max(lon1, lon2) + deg_lon
        return min_lat, max_lat, min_lon, max_lon

    def _radius_bbox(self, lat: float, lon: float, radius_m: float) -> Tuple[float, float, float, float]:
        deg_lat = radius_m / 111_320
        deg_lon = radius_m / max(111_320 * math.cos(math.radians(lat)), 1e-6)
        return lat - deg_lat, lat + deg_lat, lon - deg_lon, lon + deg_lon

    def _query_nodes_in_bbox(self, min_lat: float, max_lat: float, min_lon: float, max_lon: float):
        conn = sqlite3.connect(self._db_path, timeout=20)
        cur = conn.cursor()
        cur.execute(
            """
            SELECT id, lat, lon
            FROM nodes
            WHERE lat BETWEEN ? AND ?
              AND lon BETWEEN ? AND ?
            """,
            (min_lat, max_lat, min_lon, max_lon),
        )
        rows = cur.fetchall()
        conn.close()
        return rows

    def _query_edges_in_bbox(self, min_lat: float, max_lat: float, min_lon: float, max_lon: float):
        conn = sqlite3.connect(self._db_path, timeout=30)
        cur = conn.cursor()
        cur.execute(
            """
            SELECT e.u, e.v, e.length
            FROM edges e
            JOIN nodes nu ON nu.id = e.u
            JOIN nodes nv ON nv.id = e.v
            WHERE nu.lat BETWEEN ? AND ?
              AND nu.lon BETWEEN ? AND ?
              AND nv.lat BETWEEN ? AND ?
              AND nv.lon BETWEEN ? AND ?
            """,
            (
                min_lat, max_lat, min_lon, max_lon,
                min_lat, max_lat, min_lon, max_lon,
            ),
        )
        rows = cur.fetchall()
        conn.close()
        return rows

    def _query_radius_graph(self, lat: float, lon: float, radius_m: float) -> Optional[nx.MultiDiGraph]:
        if not self._db_ok:
            return None

        min_lat, max_lat, min_lon, max_lon = self._radius_bbox(lat, lon, radius_m)
        node_rows = self._query_nodes_in_bbox(min_lat, max_lat, min_lon, max_lon)

        node_coords = {}
        for node_id, nlat, nlon in node_rows:
            dist = _haversine(lat, lon, float(nlat), float(nlon))
            if dist <= radius_m:
                node_coords[int(node_id)] = (float(nlat), float(nlon))

        if not node_coords:
            return None

        edge_rows = self._query_edges_in_bbox(min_lat, max_lat, min_lon, max_lon)
        return self._build_graph(node_coords, edge_rows)

    def _query_bbox_graph(
        self, min_lat: float, max_lat: float, min_lon: float, max_lon: float
    ) -> Optional[nx.MultiDiGraph]:
        if not self._db_ok:
            return None

        node_rows = self._query_nodes_in_bbox(min_lat, max_lat, min_lon, max_lon)
        if not node_rows:
            return None

        node_coords = {
            int(node_id): (float(nlat), float(nlon))
            for node_id, nlat, nlon in node_rows
        }

        edge_rows = self._query_edges_in_bbox(min_lat, max_lat, min_lon, max_lon)
        return self._build_graph(node_coords, edge_rows)

    def _build_graph(self, node_coords: dict, edge_rows) -> nx.MultiDiGraph:
        G = nx.MultiDiGraph()
        G.graph["crs"] = "epsg:4326"

        for node_id, (lat, lon) in node_coords.items():
            G.add_node(node_id, y=lat, x=lon)

        allowed = set(node_coords.keys())
        for u, v, length in edge_rows:
            u = int(u)
            v = int(v)
            if u in allowed and v in allowed:
                G.add_edge(u, v, length=float(length))

        return G


class MapRenderer:
    def __init__(self, canvas, viewport: Viewport):
        self.canvas = canvas
        self.vp = viewport
        self._tag = "map_layer"
        self._route_points: List[Tuple[float, float]] = []

    def clear_route_cache(self):
        self._route_points = []

    def set_route_coords(self, route_graph, route_nodes):
        self._route_points = []
        if route_graph is None:
            return

        for node in route_nodes:
            if node in route_graph.nodes:
                d = route_graph.nodes[node]
                self._route_points.append((d["y"], d["x"]))

    def render(self, G, route_nodes=(), position=None, destination=None):
        c = self.canvas
        c.delete(self._tag)

        if G is None:
            c.create_text(
                self.vp.w // 2,
                self.vp.h // 2,
                text="Waiting for map data...",
                fill=TEXT_MUTED,
                font=("Courier", 14),
                tags=self._tag,
            )
            return

        self._draw_edges(G)
        self._draw_nodes(G)

        if self._route_points:
            self._draw_route_from_points()
        elif route_nodes:
            self._draw_route_from_graph(G, route_nodes)

        if destination:
            self._draw_marker(destination[0], destination[1], DEST_CLR, "X")

        if position:
            self._draw_position(position[0], position[1])

    def _offscreen(self, x, y, margin=20):
        return x < -margin or x > self.vp.w + margin or y < -margin or y > self.vp.h + margin

    def _draw_edges(self, G):
        for u, v, _ in G.edges(data=True):
            if u not in G.nodes or v not in G.nodes:
                continue

            x1, y1 = self.vp.geo_to_canvas(G.nodes[u]["y"], G.nodes[u]["x"])
            x2, y2 = self.vp.geo_to_canvas(G.nodes[v]["y"], G.nodes[v]["x"])

            if self._offscreen(x1, y1) and self._offscreen(x2, y2):
                continue

            self.canvas.create_line(
                x1, y1, x2, y2,
                fill=EDGE_CLR,
                width=1.5,
                tags=self._tag,
            )

    def _draw_nodes(self, G):
        if self.vp.zoom < 3:
            return

        for _, data in G.nodes(data=True):
            cx, cy = self.vp.geo_to_canvas(data["y"], data["x"])
            if self._offscreen(cx, cy):
                continue

            r = 2
            self.canvas.create_oval(
                cx - r, cy - r, cx + r, cy + r,
                fill=NODE_CLR,
                outline="",
                tags=self._tag,
            )

    def _draw_route_from_graph(self, G, route_nodes):
        coords = []
        for node in route_nodes:
            if node in G.nodes:
                d = G.nodes[node]
                cx, cy = self.vp.geo_to_canvas(d["y"], d["x"])
                coords.extend([cx, cy])

        if len(coords) >= 4:
            self.canvas.create_line(
                *coords,
                fill=ROUTE_CLR,
                width=4,
                capstyle="round",
                joinstyle="round",
                tags=self._tag,
            )

    def _draw_route_from_points(self):
        coords = []
        for lat, lon in self._route_points:
            cx, cy = self.vp.geo_to_canvas(lat, lon)
            coords.extend([cx, cy])

        if len(coords) >= 4:
            self.canvas.create_line(
                *coords,
                fill=ROUTE_CLR,
                width=4,
                capstyle="round",
                joinstyle="round",
                tags=self._tag,
            )

    def _draw_position(self, lat, lon):
        cx, cy = self.vp.geo_to_canvas(lat, lon)
        for r, color in [(18, "#0d1117"), (14, "#1a6b29"), (9, ACCENT)]:
            self.canvas.create_oval(
                cx - r, cy - r, cx + r, cy + r,
                fill=color,
                outline=ACCENT,
                width=2,
                tags=self._tag,
            )

    def _draw_marker(self, lat, lon, color, symbol):
        cx, cy = self.vp.geo_to_canvas(lat, lon)
        r = 10
        self.canvas.create_oval(
            cx - r, cy - r, cx + r, cy + r,
            fill=color,
            outline="white",
            width=2,
            tags=self._tag,
        )
        self.canvas.create_text(
            cx, cy,
            text=symbol,
            fill="white",
            font=("TkDefaultFont", 9, "bold"),
            tags=self._tag,
        )
