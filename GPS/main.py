# main.py — PiGPS Navigator  (Tkinter UI)
#
# Run:  python3 main.py
#
# Touch-screen friendly layout for 1024×600 display.
# Left panel: interactive map canvas
# Right panel: controls, status, address input, algorithm selector

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import logging
import math
import time
from typing import Optional, List, Tuple

from config import (
    WINDOW_TITLE, WINDOW_W, WINDOW_H, MAP_PANEL_W, SIDEBAR_W,
    BG_DARK, BG_MID, BG_LIGHT, ACCENT, ACCENT2, TEXT_PRIMARY, TEXT_MUTED,
    DEST_CLR, POSITION_CLR, ROUTE_CLR,
    ALGORITHMS, DEFAULT_ALGO,
    GPS_POLL_MS, MAP_REFRESH_MS, REROUTE_DIST_M,
    GRAPH_RADIUS_M,
)
from gps_reader import GPSReader
from map_engine import GraphLoader, MapRenderer, Viewport
from routing import find_path

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger(__name__)


# ─────────────────────────────────────────────────────────────────────────────
# Main Application
# ─────────────────────────────────────────────────────────────────────────────

class PiGPSApp:

    def __init__(self, root: tk.Tk):
        self.root = root
        root.title(WINDOW_TITLE)
        root.geometry(f"{WINDOW_W}x{WINDOW_H}")
        root.configure(bg=BG_DARK)
        root.resizable(False, False)

        # ── State ─────────────────────────────────────────────────────────────
        self.gps          = GPSReader()
        self.loader       = GraphLoader()
        self.viewport     = Viewport(MAP_PANEL_W, WINDOW_H)
        self.renderer: Optional[MapRenderer] = None

        self.route_nodes: List[int] = []
        self.dest_latlon: Optional[Tuple[float, float]] = None
        self.dest_node:   Optional[int]  = None
        self.route_dist_m: float  = 0.0
        self.route_eta_s:  float  = 0.0

        self._graph_loaded    = False
        self._routing_active  = False
        self._last_reroute_pos: Optional[Tuple[float, float]] = None
        self._pan_start: Optional[Tuple[int, int]] = None

        # ── Build UI ──────────────────────────────────────────────────────────
        self._build_ui()
        self._apply_styles()

        # ── Start GPS & loops ─────────────────────────────────────────────────
        self.gps.start()
        self.root.after(GPS_POLL_MS,    self._gps_poll)
        self.root.after(MAP_REFRESH_MS, self._map_refresh)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._set_status("Waiting for GPS fix…", "warning")

    # ─────────────────────────────────────────────────────────────────────────
    # UI Construction
    # ─────────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        # ── Map canvas (left) ──────────────────────────────────────────────
        self.canvas = tk.Canvas(
            self.root,
            width=MAP_PANEL_W, height=WINDOW_H,
            bg=BG_DARK, highlightthickness=0,
        )
        self.canvas.place(x=0, y=0)
        self.renderer = MapRenderer(self.canvas, self.viewport)

        # Mouse interactions
        self.canvas.bind("<ButtonPress-1>",   self._pan_start)
        self.canvas.bind("<B1-Motion>",       self._pan_drag)
        self.canvas.bind("<ButtonRelease-1>", self._pan_end)
        self.canvas.bind("<MouseWheel>",      self._mouse_zoom)
        self.canvas.bind("<Button-4>",        lambda e: self._zoom(1))
        self.canvas.bind("<Button-5>",        lambda e: self._zoom(-1))

        # ── Sidebar (right) ────────────────────────────────────────────────
        sidebar = tk.Frame(self.root, bg=BG_MID, width=SIDEBAR_W)
        sidebar.place(x=MAP_PANEL_W, y=0, width=SIDEBAR_W, height=WINDOW_H)

        y = 14

        # Title
        tk.Label(
            sidebar, text="PiGPS", font=("Courier", 20, "bold"),
            fg=ACCENT, bg=BG_MID,
        ).place(x=14, y=y)
        tk.Label(
            sidebar, text="Navigator", font=("Courier", 10),
            fg=TEXT_MUTED, bg=BG_MID,
        ).place(x=14, y=y + 26)

        y += 60

        # ── GPS Status box ────────────────────────────────────────────────
        self._gps_frame = tk.Frame(sidebar, bg=BG_LIGHT, bd=0)
        self._gps_frame.place(x=10, y=y, width=SIDEBAR_W - 20, height=76)

        self._lbl_fix = tk.Label(
            self._gps_frame, text="NO FIX", font=("Courier", 9, "bold"),
            fg=DEST_CLR, bg=BG_LIGHT,
        )
        self._lbl_fix.place(x=8, y=6)

        self._lbl_coords = tk.Label(
            self._gps_frame, text="---.----  ---.----",
            font=("Courier", 8), fg=TEXT_MUTED, bg=BG_LIGHT,
        )
        self._lbl_coords.place(x=8, y=24)

        self._lbl_speed = tk.Label(
            self._gps_frame, text="0.0 km/h  |  0 sats",
            font=("Courier", 8), fg=TEXT_MUTED, bg=BG_LIGHT,
        )
        self._lbl_speed.place(x=8, y=42)

        self._lbl_heading = tk.Label(
            self._gps_frame, text="Heading: ---°",
            font=("Courier", 8), fg=TEXT_MUTED, bg=BG_LIGHT,
        )
        self._lbl_heading.place(x=8, y=58)

        y += 88

        # ── Algorithm selector ────────────────────────────────────────────
        tk.Label(
            sidebar, text="ALGORITHM", font=("Courier", 8, "bold"),
            fg=TEXT_MUTED, bg=BG_MID,
        ).place(x=14, y=y)
        y += 18

        self._algo_var = tk.StringVar(value=DEFAULT_ALGO)
        for algo in ALGORITHMS:
            rb = tk.Radiobutton(
                sidebar, text=algo, variable=self._algo_var,
                value=algo,
                font=("Courier", 9), fg=TEXT_PRIMARY, bg=BG_MID,
                selectcolor=BG_LIGHT, activebackground=BG_MID,
                activeforeground=ACCENT,
            )
            rb.place(x=14, y=y)
            y += 20

        y += 10

        # ── Destination input ──────────────────────────────────────────────
        tk.Label(
            sidebar, text="DESTINATION", font=("Courier", 8, "bold"),
            fg=TEXT_MUTED, bg=BG_MID,
        ).place(x=14, y=y)
        y += 18

        self._dest_entry = tk.Entry(
            sidebar, font=("Courier", 9),
            bg=BG_LIGHT, fg=TEXT_PRIMARY, insertbackground=ACCENT,
            bd=0, highlightthickness=1, highlightcolor=ACCENT,
            highlightbackground=BG_LIGHT,
        )
        self._dest_entry.place(x=10, y=y, width=SIDEBAR_W - 20, height=28)
        self._dest_entry.bind("<Return>", lambda e: self._do_route())
        y += 34

        # Load Map button
        btn_load = tk.Button(
            sidebar, text="⬇  Load Map",
            font=("Courier", 9, "bold"),
            fg=BG_DARK, bg=ACCENT, activebackground=ACCENT2,
            bd=0, padx=4, pady=4, cursor="hand2",
            command=self._load_map_around_gps,
        )
        btn_load.place(x=10, y=y, width=SIDEBAR_W - 20, height=32)
        y += 38

        # Route button
        self._btn_route = tk.Button(
            sidebar, text="▶  Calculate Route",
            font=("Courier", 9, "bold"),
            fg=BG_DARK, bg=ACCENT2, activebackground=ACCENT,
            bd=0, padx=4, pady=4, cursor="hand2",
            command=self._do_route,
        )
        self._btn_route.place(x=10, y=y, width=SIDEBAR_W - 20, height=32)
        y += 38

        # Clear route button
        btn_clr = tk.Button(
            sidebar, text="✕  Clear Route",
            font=("Courier", 9),
            fg=TEXT_MUTED, bg=BG_LIGHT, activebackground=BG_DARK,
            bd=0, padx=4, pady=4, cursor="hand2",
            command=self._clear_route,
        )
        btn_clr.place(x=10, y=y, width=SIDEBAR_W - 20, height=28)
        y += 36

        # ── Route info box ────────────────────────────────────────────────
        self._info_frame = tk.Frame(sidebar, bg=BG_LIGHT)
        self._info_frame.place(x=10, y=y, width=SIDEBAR_W - 20, height=64)

        self._lbl_dist = tk.Label(
            self._info_frame, text="Distance: --",
            font=("Courier", 9), fg=ACCENT2, bg=BG_LIGHT,
        )
        self._lbl_dist.place(x=8, y=6)

        self._lbl_eta = tk.Label(
            self._info_frame, text="ETA: --",
            font=("Courier", 9), fg=ACCENT2, bg=BG_LIGHT,
        )
        self._lbl_eta.place(x=8, y=24)

        self._lbl_algo_used = tk.Label(
            self._info_frame, text="Algorithm: --",
            font=("Courier", 8), fg=TEXT_MUTED, bg=BG_LIGHT,
        )
        self._lbl_algo_used.place(x=8, y=44)

        y += 72

        # ── Zoom buttons ──────────────────────────────────────────────────
        zf = tk.Frame(sidebar, bg=BG_MID)
        zf.place(x=10, y=y, width=SIDEBAR_W - 20, height=34)

        for txt, fn, side in [("  +  ", self.viewport.zoom_in, "left"),
                               ("  −  ", self.viewport.zoom_out, "right")]:
            tk.Button(
                zf, text=txt, font=("Courier", 12, "bold"),
                fg=TEXT_PRIMARY, bg=BG_LIGHT, activebackground=BG_DARK,
                bd=0, cursor="hand2", command=fn,
            ).pack(side=side, expand=True, fill="both", padx=2)

        y += 42

        # Center-on-GPS button
        tk.Button(
            sidebar, text="⊙  Re-centre",
            font=("Courier", 9),
            fg=TEXT_PRIMARY, bg=BG_LIGHT, activebackground=BG_DARK,
            bd=0, cursor="hand2",
            command=self._centre_on_gps,
        ).place(x=10, y=y, width=SIDEBAR_W - 20, height=28)

        y += 38

        # ── Status bar ────────────────────────────────────────────────────
        self._status_var = tk.StringVar(value="Starting…")
        self._status_lbl = tk.Label(
            sidebar, textvariable=self._status_var,
            font=("Courier", 8), fg=ACCENT, bg=BG_MID,
            wraplength=SIDEBAR_W - 20, justify="left",
        )
        self._status_lbl.place(x=10, y=WINDOW_H - 40, width=SIDEBAR_W - 20)

        # ── Overlay on canvas ─────────────────────────────────────────────
        # Scale bar
        self._scale_lbl = tk.Label(
            self.canvas, text="", font=("Courier", 8),
            fg=TEXT_MUTED, bg=BG_DARK,
        )
        self._scale_lbl.place(x=10, y=WINDOW_H - 22)

        # Loading spinner label
        self._loading_lbl = tk.Label(
            self.canvas, text="", font=("Courier", 10, "bold"),
            fg=ACCENT, bg=BG_DARK,
        )
        self._loading_lbl.place(x=MAP_PANEL_W // 2 - 80, y=WINDOW_H // 2 - 12)

    def _apply_styles(self):
        style = ttk.Style()
        style.theme_use("clam")

    # ─────────────────────────────────────────────────────────────────────────
    # GPS Polling
    # ─────────────────────────────────────────────────────────────────────────

    def _gps_poll(self):
        lat, lon = self.gps.position

        if lat and lon:
            self._lbl_fix.config(
                text="GPS FIX ●" if self.gps.has_fix else "APPROX",
                fg=ACCENT if self.gps.has_fix else "#f0a500",
            )
            self._lbl_coords.config(text=f"{lat:.5f}  {lon:.5f}")
            self._lbl_speed.config(
                text=f"{self.gps.speed_kmh:.1f} km/h  |  {self.gps.satellites} sats"
            )
            self._lbl_heading.config(text=f"Heading: {self.gps.heading:.0f}°")

            # Auto-load map on first fix
            if not self._graph_loaded:
                self._load_map_around_gps()

            # Check reroute
            if self.route_nodes and self._last_reroute_pos:
                d = self._haversine(lat, lon,
                                    self._last_reroute_pos[0],
                                    self._last_reroute_pos[1])
                if d > REROUTE_DIST_M:
                    self._check_off_route(lat, lon)
        else:
            self._lbl_fix.config(text="NO FIX", fg=DEST_CLR)

        self.root.after(GPS_POLL_MS, self._gps_poll)

    # ─────────────────────────────────────────────────────────────────────────
    # Map Refresh
    # ─────────────────────────────────────────────────────────────────────────

    def _map_refresh(self):
        lat, lon = self.gps.position
        pos = (lat, lon) if (lat and lon) else None

        self.renderer.render(
            G           = self.loader.G,
            route_nodes = self.route_nodes,
            position    = pos,
            destination = self.dest_latlon,
        )
        self._update_scale_bar()
        self.root.after(MAP_REFRESH_MS, self._map_refresh)

    # ─────────────────────────────────────────────────────────────────────────
    # Map Loading
    # ─────────────────────────────────────────────────────────────────────────

    def _load_map_around_gps(self):
        lat, lon = self.gps.position
        if lat is None:
            self._set_status("No GPS fix yet — cannot load map.", "error")
            return

        self._set_status("Downloading map data…", "info")
        self._loading_lbl.config(text="Fetching map…")
        self._graph_loaded = False

        self.loader.load(
            lat, lon,
            radius_m=GRAPH_RADIUS_M,
            on_done=self._on_graph_loaded,
            on_error=self._on_graph_error,
        )

    def _on_graph_loaded(self, G):
        self._graph_loaded = True
        self.viewport.fit_graph(G)
        # Centre on GPS
        lat, lon = self.gps.position
        if lat:
            self.viewport.center_lat = lat
            self.viewport.center_lon = lon
        self._loading_lbl.config(text="")
        self._set_status(
            f"Map loaded: {G.number_of_nodes()} nodes, {G.number_of_edges()} edges",
            "ok"
        )

    def _on_graph_error(self, msg):
        self._loading_lbl.config(text="")
        self._set_status(f"Map error: {msg}", "error")

    # ─────────────────────────────────────────────────────────────────────────
    # Routing
    # ─────────────────────────────────────────────────────────────────────────

    def _do_route(self):
        if self.loader.G is None:
            self._set_status("Load map first.", "error")
            return

        dest_txt = self._dest_entry.get().strip()
        if not dest_txt:
            self._set_status("Enter a destination address.", "error")
            return

        lat, lon = self.gps.position
        if lat is None:
            self._set_status("No GPS fix.", "error")
            return

        if self._routing_active:
            return
        self._routing_active = True
        self._btn_route.config(state="disabled", text="Routing…")
        self._set_status("Geocoding destination…", "info")

        algo = self._algo_var.get()

        def _worker():
            try:
                # Geocode
                dest_ll = self.loader.geocode(dest_txt)
                if dest_ll is None:
                    self.root.after(0, lambda: self._set_status(
                        "Could not geocode address.", "error"))
                    return

                d_lat, d_lon = dest_ll
                orig_node = self.loader.nearest_node(lat, lon)
                dest_node = self.loader.nearest_node(d_lat, d_lon)

                if orig_node is None or dest_node is None:
                    self.root.after(0, lambda: self._set_status(
                        "Could not find road nodes.", "error"))
                    return

                self.root.after(0, lambda: self._set_status(
                    f"Running {algo}…", "info"))

                t0    = time.time()
                path  = find_path(self.loader.G, orig_node, dest_node, algorithm=algo)
                t_ms  = (time.time() - t0) * 1000

                if not path:
                    self.root.after(0, lambda: self._set_status(
                        "No route found.", "error"))
                    return

                # Compute distance & ETA
                dist_m = self._path_length(path)
                avg_spd = max(self.gps.speed_kmh, 30.0)   # km/h
                eta_s   = (dist_m / 1000) / avg_spd * 3600

                def _apply():
                    self.route_nodes      = path
                    self.dest_latlon      = (d_lat, d_lon)
                    self.dest_node        = dest_node
                    self._last_reroute_pos = (lat, lon)
                    self._lbl_dist.config(text=f"Distance: {dist_m/1000:.2f} km")
                    eta_str = f"{int(eta_s//60)} min {int(eta_s%60)} s"
                    self._lbl_eta.config(text=f"ETA: {eta_str}")
                    self._lbl_algo_used.config(text=f"Algorithm: {algo}  ({t_ms:.0f}ms)")
                    self._set_status(f"Route ready — {len(path)} nodes", "ok")
                self.root.after(0, _apply)

            except Exception as e:
                logger.exception("Routing error")
                self.root.after(0, lambda: self._set_status(f"Error: {e}", "error"))
            finally:
                self._routing_active = False
                self.root.after(0, lambda: self._btn_route.config(
                    state="normal", text="▶  Calculate Route"))

        threading.Thread(target=_worker, daemon=True).start()

    def _check_off_route(self, lat: float, lon: float):
        """Re-route if user has deviated significantly."""
        if self.loader.G is None or self.dest_node is None:
            return
        self._last_reroute_pos = (lat, lon)
        orig_node = self.loader.nearest_node(lat, lon)
        if orig_node is None:
            return
        algo  = self._algo_var.get()
        path  = find_path(self.loader.G, orig_node, self.dest_node, algorithm=algo)
        if path:
            self.route_nodes = path
            self._set_status("Re-routed.", "ok")

    def _clear_route(self):
        self.route_nodes  = []
        self.dest_latlon  = None
        self.dest_node    = None
        self._lbl_dist.config(text="Distance: --")
        self._lbl_eta.config(text="ETA: --")
        self._lbl_algo_used.config(text="Algorithm: --")
        self._set_status("Route cleared.", "info")

    def _path_length(self, path: list) -> float:
        G = self.loader.G
        total = 0.0
        for u, v in zip(path[:-1], path[1:]):
            data = G.get_edge_data(u, v)
            if data:
                lengths = [d.get("length", 0) for d in data.values()]
                total += min(lengths)
        return total

    # ─────────────────────────────────────────────────────────────────────────
    # Pan & Zoom
    # ─────────────────────────────────────────────────────────────────────────

    def _pan_start(self, event):
        self._pan_start_xy = (event.x, event.y)
        self._pan_start_ll = (self.viewport.center_lat, self.viewport.center_lon)

    def _pan_drag(self, event):
        if not hasattr(self, "_pan_start_xy"):
            return
        sx, sy = self._pan_start_xy
        dx, dy = event.x - sx, event.y - sy
        m_per_deg_lat = 111_320
        m_per_deg_lon = 111_320 * math.cos(math.radians(self.viewport.center_lat))
        scale = self.viewport._base_scale * self.viewport.zoom
        if scale == 0:
            return
        dlat =  dy / (m_per_deg_lat * scale)
        dlon = -dx / (m_per_deg_lon * scale)
        start_lat, start_lon = self._pan_start_ll
        self.viewport.center_lat = start_lat + dlat
        self.viewport.center_lon = start_lon + dlon

    def _pan_end(self, event):
        pass

    def _mouse_zoom(self, event):
        if event.delta > 0:
            self.viewport.zoom_in()
        else:
            self.viewport.zoom_out()

    def _zoom(self, direction: int):
        if direction > 0:
            self.viewport.zoom_in()
        else:
            self.viewport.zoom_out()

    def _centre_on_gps(self):
        lat, lon = self.gps.position
        if lat:
            self.viewport.center_lat = lat
            self.viewport.center_lon = lon

    # ─────────────────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _set_status(self, msg: str, level: str = "info"):
        colours = {"ok": ACCENT, "info": ACCENT2, "warning": "#f0a500", "error": DEST_CLR}
        self._status_var.set(msg)
        self._status_lbl.config(fg=colours.get(level, TEXT_MUTED))
        logger.info(f"[{level}] {msg}")

    def _update_scale_bar(self):
        if self.viewport._base_scale == 0:
            return
        # Width of 100 px in metres
        m_per_px = 1 / (self.viewport._base_scale * self.viewport.zoom)
        m_100 = m_per_px * 100
        if m_100 >= 1000:
            label = f"← 100px = {m_100/1000:.1f} km"
        else:
            label = f"← 100px = {m_100:.0f} m"
        self._scale_lbl.config(text=label)

    @staticmethod
    def _haversine(lat1, lon1, lat2, lon2) -> float:
        R = 6_371_000
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlat, dlon = lat2 - lat1, lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
        return R * 2 * math.asin(math.sqrt(a))

    def _on_close(self):
        self.gps.stop()
        self.root.destroy()


# ─────────────────────────────────────────────────────────────────────────────
# Entry Point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    root = tk.Tk()
    app  = PiGPSApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
