# map_engine.py — SQLite-backed graph loader & Tkinter canvas renderer

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
    CACHE_DIR, GRAPH_RADIUS_M, NETWORK_TYPE,
    NODE_CLR, EDGE_CLR, ROUTE_CLR, POSITION_CLR, DEST_CLR,
    ACCENT, ACCENT2, TEXT_MUTED,
    SUBGRAPH_RADIUS_M, ROUTING_RADIUS_M,
    GRAPH_DB,
)

logger = logging.getLogger(__name__)
os.makedirs(CACHE_DIR, exist_ok=True)


# ─────────────────────────────────────────────────────────────────────────────
# Viewport
# ─────────────────────────────────────────────────────────────────────────────

class Viewport:
    def __init__(self, canvas_w: int, canvas_h: int):
        self.w           = canvas_w
        self.h           = canvas_h
        self.center_lat  = 36.18
        self.center_lon  = -85.50
        self.zoom        = 1.0
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
        width_m  = lon_span * m_per_deg_lon
        scale_h  = self.h / height_m if height_m else 1
        scale_w  = self.w / width_m  if width_m  else 1
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

    def zoom_in(self):
        self.zoom = min(self.zoom * 1.3, 20.0)

    def zoom_out(self):
        self.zoom = max(self.zoom / 1.3, 0.1)


# ─────────────────────────────────────────────────────────────────────────────
# SQLite Graph Loader
# ─────────────────────────────────────────────────────────────────────────────

class GraphLoader:
    """
    Loads graph data on-demand from SQLite.
    Never loads the full graph into RAM — only the local area around the user.
    Full-graph routing uses the DB directly via a lazy-loaded NetworkX graph.
    """

    def __init__(self):
        self.G: Optional[nx.MultiDiGraph]      = None  # display subgraph
        self.G_full: Optional[nx.MultiDiGraph] = None  # routing graph (lazy)
        self._lock    = threading.Lock()
        self._db_path = GRAPH_DB
        self._db_ok   = os.path.exists(self._db_path)
        self._loading = False

        if not self._db_ok:
            logger.error(f"Graph DB not found: {self._db_path}")

        if OSMNX_OK:
            ox.settings.use_cache    = True
            ox.settings.cache_folder = CACHE_DIR
            ox.settings.log_console  = False

    # ── Public API ────────────────────────────────────────────────────────────

    def load(self, lat: float, lon: float,
             radius_m: int = SUBGRAPH_RADIUS_M,
             on_done=None, on_error=None):
        """Entry point called by main.py — loads subgraph then routing graph."""
        def _worker():
            try:
                print(f"[LOADER] Loading subgraph around ({lat:.4f},{lon:.4f})")
                sub = self._query_subgraph(lat, lon, SUBGRAPH_RADIUS_M)
                if sub is None or sub.number_of_nodes() == 0:
                    raise RuntimeError("No nodes found near GPS position")

                with self._lock:
                    self.G = sub

                print(f"[LOADER] Subgraph: {sub.number_of_nodes()} nodes")

                # Load routing graph in background
                threading.Thread(
                    target=self._load_routing_graph,
                    daemon=True
                ).start()

                if on_done:
                    on_done(sub)

            except Exception as e:
                import traceback
                traceback.print_exc()
                logger.error(f"Load failed: {e}")
                if on_error:
                    on_error(str(e))

        threading.Thread(target=_worker, daemon=True).start()

    def update_subgraph(self, lat: float, lon: float):
        """Called every GPS poll — refreshes display subgraph around user."""
        def _bg():
            sub = self._query_subgraph(lat, lon, SUBGRAPH_RADIUS_M)
            if sub and sub.number_of_nodes() > 0:
                with self._lock:
                    self.G = sub
                    self._center = (lat, lon)
        threading.Thread(target=_bg, daemon=True).start()

    def nearest_node(self, lat: float, lon: float,
                     use_full: bool = True) -> Optional[int]:
        """
        Find nearest node using direct SQLite spatial query.
        No scikit-learn required — pure SQL distance calculation.
        """
        if not self._db_ok:
            return None
        try:
            deg_margin = 0.05
            conn = sqlite3.connect(self._db_path, timeout=10)
            c    = conn.cursor()

            # Bounding box pre-filter then sort by approximate distance
            c.execute("""
                SELECT id,
                       ((lat - ?) * (lat - ?) * 111320 * 111320 +
                        (lon - ?) * (lon - ?) * 85000  * 85000) AS dist2
                FROM nodes
                WHERE lat BETWEEN ? AND ?
                  AND lon BETWEEN ? AND ?
                ORDER BY dist2
                LIMIT 1
            """, (
                lat, lat, lon, lon,
                lat - deg_margin, lat + deg_margin,
                lon - deg_margin, lon + deg_margin,
            ))
            row = conn.fetchone() if False else c.fetchone()
            conn.close()
            if row:
                logger.debug(f"Nearest node to ({lat:.4f},{lon:.4f}): {row[0]}")
                return int(row[0])
            return None
        except Exception as e:
            logger.error(f"nearest_node error: {e}")
            return None

    def geocode(self, address: str) -> Optional[Tuple[float, float]]:
        """
        Try multiple geocoding strategies to maximise success rate.
        Falls back through progressively simpler query formats.
        """
        if not OSMNX_OK:
            return None

        # Build a list of queries to try in order
        queries = [address]

        # If address doesn't mention TN, append it
        if "TN" not in address.upper() and "tennessee" not in address.lower():
            queries.append(f"{address}, Tennessee, USA")

        # If it looks like a street address, try city-only fallback
        parts = [p.strip() for p in address.split(",")]
        if len(parts) >= 2:
            # Try just the last two parts (city, state)
            queries.append(", ".join(parts[-2:]))
            # Try appending Tennessee
            queries.append(f"{parts[0]}, Tennessee, USA")

        # Always try with full state name appended
        queries.append(f"{address}, Tennessee")

        for query in queries:
            try:
                result = ox.geocode(query)
                if result:
                    lat, lon = result
                    # Validate result is within our graph bounding box
                    # with a generous margin
                    if 35.5 <= lat <= 36.8 and -87.0 <= lon <= -85.0:
                        logger.info(f"Geocoded '{query}' → {lat:.4f},{lon:.4f}")
                        return result
                    else:
                        logger.warning(
                            f"Geocode result for '{query}' is outside Tennessee"
                            f" ({lat:.4f},{lon:.4f}) — skipping"
                        )
            except Exception as e:
                logger.debug(f"Geocode attempt failed for '{query}': {e}")
                continue

        logger.error(f"All geocode attempts failed for '{address}'")
        return None

    # ── Internal ──────────────────────────────────────────────────────────────

    def _query_subgraph(self, lat: float, lon: float,
                        radius_m: float) -> Optional[nx.MultiDiGraph]:
        """
        Query SQLite for nodes within radius_m and their edges.
        Returns a small NetworkX graph — typically 200-500 nodes.
        """
        if not self._db_ok:
            return None

        # Convert radius to degree margin for bounding box pre-filter
        deg_margin = (radius_m / 111_320) * 1.5

        try:
            conn = sqlite3.connect(self._db_path, timeout=10)
            conn.row_factory = sqlite3.Row
            c = conn.cursor()

            # Get nodes in bounding box
            c.execute("""
                SELECT id, lat, lon FROM nodes
                WHERE lat BETWEEN ? AND ?
                  AND lon BETWEEN ? AND ?
            """, (
                lat - deg_margin, lat + deg_margin,
                lon - deg_margin, lon + deg_margin,
            ))
            rows = c.fetchall()

            if not rows:
                conn.close()
                return None

            # Exact haversine filter
            R = 6_371_000
            node_ids = []
            node_coords = {}
            for row in rows:
                node_lat, node_lon = row["lat"], row["lon"]
                dlat = math.radians(node_lat - lat)
                dlon = math.radians(node_lon - lon)
                a = (math.sin(dlat/2)**2 +
                     math.cos(math.radians(lat)) *
                     math.cos(math.radians(node_lat)) *
                     math.sin(dlon/2)**2)
                dist = R * 2 * math.asin(math.sqrt(a))
                if dist <= radius_m:
                    nid = int(row["id"])
                    node_ids.append(nid)
                    node_coords[nid] = (node_lat, node_lon)

            if not node_ids:
                conn.close()
                return None

            # Get edges between nodes in subgraph
            placeholders = ",".join("?" * len(node_ids))
            c.execute(f"""
                SELECT u, v, length FROM edges
                WHERE u IN ({placeholders})
                  AND v IN ({placeholders})
            """, node_ids + node_ids)
            edge_rows = c.fetchall()
            conn.close()

            # Build NetworkX graph
            G = nx.MultiDiGraph()
            G.graph["crs"] = "epsg:4326"

            for nid, (nlat, nlon) in node_coords.items():
                G.add_node(nid, y=nlat, x=nlon)

            for row in edge_rows:
                G.add_edge(int(row["u"]), int(row["v"]),
                           length=float(row["length"]))

            return G

        except Exception as e:
            logger.error(f"_query_subgraph error: {e}")
            return None

    def _load_routing_graph(self):
        """Load full routing graph from SQLite in one fast pass."""
        import time as _time
        print("[LOADER] Loading routing graph...")
        t0 = _time.time()

        try:
            conn = sqlite3.connect(self._db_path, timeout=30)
            conn.execute("PRAGMA journal_mode=OFF")
            conn.execute("PRAGMA synchronous=OFF")
            conn.execute("PRAGMA cache_size=10000")
            conn.execute("PRAGMA temp_store=MEMORY")
            c = conn.cursor()

            c.execute("SELECT id, lat, lon FROM nodes")
            nodes = c.fetchall()
            print(f"[LOADER] {len(nodes)} nodes in {_time.time()-t0:.1f}s")

            t1 = _time.time()
            c.execute("SELECT u, v, length FROM edges")
            edges = c.fetchall()
            print(f"[LOADER] {len(edges)} edges in {_time.time()-t1:.1f}s")
            conn.close()

            t2 = _time.time()
            G = nx.MultiDiGraph()
            G.graph["crs"] = "epsg:4326"
            G.add_nodes_from(
                (int(r[0]), {"y": float(r[1]), "x": float(r[2])})
                for r in nodes
            )
            G.add_edges_from(
                (int(r[0]), int(r[1]), {"length": float(r[2])})
                for r in edges
            )
            print(f"[LOADER] Graph built in {_time.time()-t2:.1f}s")
            print(f"[LOADER] Total: {_time.time()-t0:.1f}s")

            import resource
            ram = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
            print(f"[LOADER] RAM: {ram:.0f} MB")

            with self._lock:
                self.G_full = G

        except Exception as e:
            logger.error(f"Routing graph load failed: {e}")
            import traceback
            traceback.print_exc()


# ─────────────────────────────────────────────────────────────────────────────
# Map Renderer
# ─────────────────────────────────────────────────────────────────────────────

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
        # Route may include nodes outside the display subgraph
        # so we need to handle missing nodes gracefully
        coords = []
        for node in route_nodes:
            if node in G.nodes:
                d = G.nodes[node]
                cx, cy = self.vp.geo_to_canvas(d["y"], d["x"])
                coords.extend([cx, cy])
            elif hasattr(self, '_route_coords') and node in self._route_coords:
                cx, cy = self._route_coords[node]
                coords.extend([cx, cy])
        if len(coords) >= 4:
            self.canvas.create_line(*coords,
                fill=ROUTE_CLR, width=4,
                capstyle="round", joinstyle="round",
                tags=self._tag)

    def set_route_coords(self, G_full, route_nodes):
        """Pre-cache canvas coords for all route nodes from the full graph."""
        self._route_coords = {}
        if G_full is None:
            return
        for node in route_nodes:
            if node in G_full.nodes:
                d = G_full.nodes[node]
                self._route_coords[node] = self.vp.geo_to_canvas(
                    d["y"], d["x"])

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