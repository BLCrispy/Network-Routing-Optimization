import tkinter as tk
import threading
import logging
import math
import time
from typing import Optional, List, Tuple

from config import (
    WINDOW_TITLE,
    WINDOW_W,
    WINDOW_H,
    MAP_PANEL_W,
    SIDEBAR_W,
    BG_DARK,
    BG_MID,
    BG_LIGHT,
    ACCENT,
    ACCENT2,
    TEXT_PRIMARY,
    TEXT_MUTED,
    DEST_CLR,
    ALGORITHMS,
    DEFAULT_ALGO,
    GPS_POLL_MS,
    MAP_REFRESH_MS,
    REROUTE_DIST_M,
    SUBGRAPH_RADIUS_M,
    DISPLAY_REFRESH_MIN_MOVE_M,
    LOCAL_ROUTE_MAX_M,
    LONG_ROUTE_MARGIN_STEPS_M,
    LOCAL_ONLY_ALGOS,
)
from gps_reader import GPSReader
from map_engine import GraphLoader, MapRenderer, Viewport
from routing import find_path

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger(__name__)


class OSKKeyboard(tk.Toplevel):
    ROWS = [
        ["1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "BK"],
        ["q", "w", "e", "r", "t", "y", "u", "i", "o", "p"],
        ["a", "s", "d", "f", "g", "h", "j", "k", "l", "'"],
        ["z", "x", "c", "v", "b", "n", "m", ",", ".", "ENT"],
        ["@", "#", "-", "_", "(", ")", "/", "\\", "!", "?"],
    ]

    def __init__(self, parent, target_entry: tk.Entry):
        super().__init__(parent)
        self.target = target_entry
        self.caps = False

        self.overrideredirect(True)
        self.attributes("-topmost", True)
        self.configure(bg=BG_MID)

        self._build()
        self._position()

    def _build(self):
        for r, row in enumerate(self.ROWS):
            for c, key in enumerate(row):
                tk.Button(
                    self,
                    text=key,
                    font=("Courier", 10, "bold"),
                    fg=TEXT_PRIMARY,
                    bg=BG_LIGHT,
                    activebackground=ACCENT,
                    activeforeground=BG_DARK,
                    relief="flat",
                    bd=1,
                    width=4,
                    command=lambda k=key: self._press(k),
                ).grid(row=r, column=c, padx=2, pady=2, sticky="nsew")

        bottom = tk.Frame(self, bg=BG_MID)
        bottom.grid(row=len(self.ROWS), column=0, columnspan=11, sticky="ew", padx=2, pady=2)

        self._caps_btn = tk.Button(
            bottom,
            text="CAPS",
            font=("Courier", 10, "bold"),
            fg=TEXT_PRIMARY,
            bg=BG_LIGHT,
            activebackground=ACCENT2,
            relief="flat",
            bd=1,
            width=6,
            command=self._toggle_caps,
        )
        self._caps_btn.pack(side="left", padx=2)

        tk.Button(
            bottom,
            text="SPACE",
            font=("Courier", 10, "bold"),
            fg=TEXT_PRIMARY,
            bg=BG_LIGHT,
            activebackground=ACCENT,
            activeforeground=BG_DARK,
            relief="flat",
            bd=1,
            width=14,
            command=lambda: self.target.insert(tk.END, " "),
        ).pack(side="left", padx=2, expand=True, fill="x")

        tk.Button(
            bottom,
            text="CLOSE",
            font=("Courier", 10, "bold"),
            fg=TEXT_PRIMARY,
            bg=DEST_CLR,
            activebackground="#da3633",
            relief="flat",
            bd=1,
            width=8,
            command=self.destroy,
        ).pack(side="right", padx=2)

    def _press(self, key: str):
        if key == "BK":
            cur = self.target.get()
            self.target.delete(0, tk.END)
            self.target.insert(0, cur[:-1])
        elif key == "ENT":
            self.target.event_generate("<Return>")
            self.destroy()
        else:
            text = key.upper() if self.caps and len(key) == 1 and key.isalpha() else key
            self.target.insert(tk.END, text)

    def _toggle_caps(self):
        self.caps = not self.caps
        self._caps_btn.config(
            bg=ACCENT2 if self.caps else BG_LIGHT,
            fg=BG_DARK if self.caps else TEXT_PRIMARY,
        )

    def _position(self):
        self.update_idletasks()
        sw = self.winfo_screenwidth()
        sh = self.winfo_screenheight()
        kw = self.winfo_reqwidth()
        kh = self.winfo_reqheight()
        x = (sw - kw) // 2
        y = sh - kh - 10
        self.geometry(f"+{x}+{y}")


class PiGPSApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title(WINDOW_TITLE)
        self.root.geometry(f"{WINDOW_W}x{WINDOW_H}")
        self.root.configure(bg=BG_DARK)
        self.root.resizable(False, False)

        self.gps = GPSReader()
        self.loader = GraphLoader()
        self.viewport = Viewport(MAP_PANEL_W, WINDOW_H)
        self.renderer = None

        self.route_nodes: List[int] = []
        self.dest_latlon: Optional[Tuple[float, float]] = None
        self.dest_node: Optional[int] = None
        self.route_graph = None
        self.route_algorithm = DEFAULT_ALGO

        self._graph_loaded = False
        self._routing_active = False
        self._user_panning = False
        self._last_reroute_pos: Optional[Tuple[float, float]] = None
        self._last_display_refresh_pos: Optional[Tuple[float, float]] = None

        self._build_ui()

        self.gps.start()
        self.root.after(GPS_POLL_MS, self._gps_poll)
        self.root.after(MAP_REFRESH_MS, self._map_refresh)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._set_status("Waiting for GPS fix...", "warning")

    def _build_ui(self):
        self.canvas = tk.Canvas(
            self.root,
            width=MAP_PANEL_W,
            height=WINDOW_H,
            bg=BG_DARK,
            highlightthickness=0,
        )
        self.canvas.place(x=0, y=0)
        self.renderer = MapRenderer(self.canvas, self.viewport)

        self.canvas.bind("<ButtonPress-1>", self._pan_start)
        self.canvas.bind("<B1-Motion>", self._pan_drag)
        self.canvas.bind("<MouseWheel>", self._mouse_zoom)
        self.canvas.bind("<Button-4>", lambda e: self._zoom(1))
        self.canvas.bind("<Button-5>", lambda e: self._zoom(-1))

        sidebar = tk.Frame(self.root, bg=BG_MID, width=SIDEBAR_W)
        sidebar.place(x=MAP_PANEL_W, y=0, width=SIDEBAR_W, height=WINDOW_H)

        y = 8

        tk.Label(
            sidebar,
            text="PiGPS Navigator",
            font=("Courier", 11, "bold"),
            fg=ACCENT,
            bg=BG_MID,
        ).place(x=8, y=y)
        y += 24

        self._gps_frame = tk.Frame(sidebar, bg=BG_LIGHT)
        self._gps_frame.place(x=6, y=y, width=SIDEBAR_W - 12, height=64)

        self._lbl_fix = tk.Label(
            self._gps_frame,
            text="NO FIX",
            font=("Courier", 8, "bold"),
            fg=DEST_CLR,
            bg=BG_LIGHT,
        )
        self._lbl_fix.place(x=6, y=3)

        self._lbl_coords = tk.Label(
            self._gps_frame,
            text="---.-----  ---.-----",
            font=("Courier", 7),
            fg=TEXT_MUTED,
            bg=BG_LIGHT,
        )
        self._lbl_coords.place(x=6, y=19)

        self._lbl_speed = tk.Label(
            self._gps_frame,
            text="0.0 km/h | 0 sats",
            font=("Courier", 7),
            fg=TEXT_MUTED,
            bg=BG_LIGHT,
        )
        self._lbl_speed.place(x=6, y=35)

        self._lbl_heading = tk.Label(
            self._gps_frame,
            text="Heading: ---",
            font=("Courier", 7),
            fg=TEXT_MUTED,
            bg=BG_LIGHT,
        )
        self._lbl_heading.place(x=6, y=49)
        y += 72

        tk.Label(
            sidebar,
            text="ALGORITHM",
            font=("Courier", 7, "bold"),
            fg=TEXT_MUTED,
            bg=BG_MID,
        ).place(x=6, y=y)
        y += 15

        self._algo_var = tk.StringVar(value=DEFAULT_ALGO)
        algo_frame = tk.Frame(sidebar, bg=BG_MID)
        algo_frame.place(x=6, y=y, width=SIDEBAR_W - 12, height=80)

        for i, algo in enumerate(ALGORITHMS):
            tk.Radiobutton(
                algo_frame,
                text=algo,
                variable=self._algo_var,
                value=algo,
                font=("Courier", 7),
                fg=TEXT_PRIMARY,
                bg=BG_MID,
                selectcolor=BG_LIGHT,
                activebackground=BG_MID,
                activeforeground=ACCENT,
            ).grid(row=i // 2, column=i % 2, sticky="w", padx=2, pady=2)

        y += 84

        tk.Label(
            sidebar,
            text="DESTINATION",
            font=("Courier", 7, "bold"),
            fg=TEXT_MUTED,
            bg=BG_MID,
        ).place(x=6, y=y)
        y += 15

        self._dest_entry = tk.Entry(
            sidebar,
            font=("Courier", 8),
            bg=BG_LIGHT,
            fg=TEXT_PRIMARY,
            insertbackground=ACCENT,
            bd=0,
            highlightthickness=1,
            highlightbackground=BG_LIGHT,
            highlightcolor=ACCENT,
        )
        self._dest_entry.place(x=6, y=y, width=SIDEBAR_W - 34, height=24)
        self._dest_entry.bind("<Return>", lambda e: self._do_route())
        self._dest_entry.bind("<FocusIn>", self._show_keyboard)

        tk.Button(
            sidebar,
            text="K",
            font=("Courier", 10),
            fg=TEXT_PRIMARY,
            bg=BG_LIGHT,
            activebackground=BG_DARK,
            bd=0,
            command=self._show_keyboard,
        ).place(x=SIDEBAR_W - 26, y=y, width=20, height=24)
        y += 30

        tk.Button(
            sidebar,
            text="Load Map",
            font=("Courier", 8, "bold"),
            fg=BG_DARK,
            bg=ACCENT,
            activebackground=ACCENT2,
            bd=0,
            command=self._load_map_around_gps,
        ).place(x=6, y=y, width=SIDEBAR_W - 12, height=24)
        y += 30

        self._btn_route = tk.Button(
            sidebar,
            text="Calculate Route",
            font=("Courier", 8, "bold"),
            fg=BG_DARK,
            bg=ACCENT2,
            activebackground=ACCENT,
            bd=0,
            command=self._do_route,
        )
        self._btn_route.place(x=6, y=y, width=SIDEBAR_W - 12, height=24)
        y += 30

        tk.Button(
            sidebar,
            text="Clear Route",
            font=("Courier", 8),
            fg=TEXT_PRIMARY,
            bg=BG_LIGHT,
            activebackground=BG_DARK,
            bd=0,
            command=self._clear_route,
        ).place(x=6, y=y, width=SIDEBAR_W - 12, height=24)
        y += 34

        info_frame = tk.Frame(sidebar, bg=BG_LIGHT)
        info_frame.place(x=6, y=y, width=SIDEBAR_W - 12, height=96)

        self._lbl_dist = tk.Label(
            info_frame,
            text="Distance: --",
            font=("Courier", 8),
            fg=TEXT_PRIMARY,
            bg=BG_LIGHT,
        )
        self._lbl_dist.place(x=6, y=8)

        self._lbl_eta = tk.Label(
            info_frame,
            text="ETA: --",
            font=("Courier", 8),
            fg=TEXT_PRIMARY,
            bg=BG_LIGHT,
        )
        self._lbl_eta.place(x=6, y=30)

        self._lbl_algo_used = tk.Label(
            info_frame,
            text="Algorithm: --",
            font=("Courier", 8),
            fg=TEXT_PRIMARY,
            bg=BG_LIGHT,
        )
        self._lbl_algo_used.place(x=6, y=52)

        self._loading_lbl = tk.Label(
            info_frame,
            text="",
            font=("Courier", 7),
            fg=ACCENT2,
            bg=BG_LIGHT,
        )
        self._loading_lbl.place(x=6, y=74)
        y += 108

        self._status_var = tk.StringVar(value="")
        self._status_lbl = tk.Label(
            sidebar,
            textvariable=self._status_var,
            font=("Courier", 7, "bold"),
            fg=TEXT_MUTED,
            bg=BG_MID,
            justify="left",
            wraplength=SIDEBAR_W - 12,
        )
        self._status_lbl.place(x=6, y=y, width=SIDEBAR_W - 12, height=48)

        self._scale_lbl = tk.Label(
            self.root,
            text="",
            font=("Courier", 7),
            fg=TEXT_MUTED,
            bg=BG_DARK,
        )
        self._scale_lbl.place(x=12, y=WINDOW_H - 24)

    def _gps_poll(self):
        lat, lon = self.gps.position

        if lat is not None and lon is not None:
            self._lbl_fix.config(
                text="GPS FIX" if self.gps.has_fix else "APPROX",
                fg=ACCENT if self.gps.has_fix else "#f0a500",
            )
            self._lbl_coords.config(text=f"{lat:.5f}  {lon:.5f}")
            self._lbl_speed.config(text=f"{self.gps.speed_kmh:.1f} km/h | {self.gps.satellites} sats")
            self._lbl_heading.config(text=f"Heading: {self.gps.heading:.0f}")

            if not self._graph_loaded:
                self._load_map_around_gps()
            elif self._should_refresh_display(lat, lon):
                self.loader.update_subgraph(lat, lon, SUBGRAPH_RADIUS_M)
                self._last_display_refresh_pos = (lat, lon)

            if self.route_nodes and self._last_reroute_pos:
                moved = self._haversine(lat, lon, self._last_reroute_pos[0], self._last_reroute_pos[1])
                if moved > REROUTE_DIST_M:
                    self._check_off_route(lat, lon)
        else:
            self._lbl_fix.config(text="NO FIX", fg=DEST_CLR)

        self.root.after(GPS_POLL_MS, self._gps_poll)

    def _map_refresh(self):
        lat, lon = self.gps.position
        pos = (lat, lon) if lat is not None and lon is not None else None

        if pos and not self._user_panning:
            self.viewport.center_lat = lat
            self.viewport.center_lon = lon

        if self._user_panning and time.time() - getattr(self, "_pan_time", 0) > 10:
            self._user_panning = False

        self.renderer.render(
            G=self.loader.G,
            route_nodes=self.route_nodes,
            position=pos,
            destination=self.dest_latlon,
        )
        self._update_scale_bar()
        self.root.after(MAP_REFRESH_MS, self._map_refresh)

    def _load_map_around_gps(self):
        lat, lon = self.gps.position
        if lat is None or lon is None:
            lat, lon = 36.1628, -85.5016

        self._set_status("Loading local display graph...", "info")
        self._loading_lbl.config(text="Loading map...")

        self.loader.load(
            lat,
            lon,
            radius_m=SUBGRAPH_RADIUS_M,
            on_done=self._on_graph_loaded,
            on_error=self._on_graph_error,
        )

    def _on_graph_loaded(self, G):
        self._graph_loaded = True
        lat, lon = self.gps.position
        if lat is None or lon is None:
            lat, lon = 36.1628, -85.5016

        self.viewport.fit_graph(G)
        self.viewport.center_lat = lat
        self.viewport.center_lon = lon
        self._last_display_refresh_pos = (lat, lon)

        self._loading_lbl.config(text="")
        self._set_status(f"Display graph loaded: {G.number_of_nodes()} nodes", "ok")

    def _on_graph_error(self, msg):
        self._loading_lbl.config(text="")
        self._set_status(f"Map error: {msg}", "error")

    def _do_route(self):
        if self.loader.G is None:
            self._set_status("Load map first.", "error")
            return

        dest_txt = self._dest_entry.get().strip()
        if not dest_txt:
            self._set_status("Enter a destination address.", "error")
            return

        lat, lon = self.gps.position
        if lat is None or lon is None:
            self._set_status("No GPS fix.", "error")
            return

        if self._routing_active:
            return

        self._routing_active = True
        self._btn_route.config(state="disabled", text="Routing...")
        self._set_status("Geocoding destination...", "info")

        algo = self._algo_var.get()

        def _worker():
            try:
                dest_ll = self.loader.geocode(dest_txt)
                if dest_ll is None:
                    self.root.after(0, lambda: self._set_status("Could not geocode address.", "error"))
                    return

                d_lat, d_lon = dest_ll
                route_result = self._compute_route(lat, lon, d_lat, d_lon, algo)

                if route_result is None:
                    return

                route_graph, path, dest_node, elapsed_ms = route_result
                dist_m = self._path_length(path, route_graph)
                avg_spd = max(self.gps.speed_kmh, 30.0)
                eta_s = (dist_m / 1000.0) / avg_spd * 3600.0

                def _apply():
                    self.route_nodes = path
                    self.route_graph = route_graph
                    self.dest_latlon = (d_lat, d_lon)
                    self.dest_node = dest_node
                    self.route_algorithm = algo
                    self._last_reroute_pos = (lat, lon)

                    self.renderer.set_route_coords(route_graph, path)
                    self._lbl_dist.config(text=f"Distance: {dist_m / 1000.0:.2f} km")
                    self._lbl_eta.config(text=f"ETA: {int(eta_s // 60)} min {int(eta_s % 60)} s")
                    self._lbl_algo_used.config(text=f"Algorithm: {algo} ({elapsed_ms:.0f} ms)")
                    self._set_status(f"Route ready - {len(path)} nodes", "ok")

                self.root.after(0, _apply)

            except Exception as e:
                logger.exception("Routing error")
                self.root.after(0, lambda: self._set_status(f"Error: {e}", "error"))
            finally:
                self._routing_active = False
                self.root.after(0, lambda: self._btn_route.config(state="normal", text="Calculate Route"))

        threading.Thread(target=_worker, daemon=True).start()

    def _compute_route(self, orig_lat, orig_lon, dest_lat, dest_lon, algo):
        straight_m = self._haversine(orig_lat, orig_lon, dest_lat, dest_lon)

        if algo in LOCAL_ONLY_ALGOS and straight_m > LOCAL_ROUTE_MAX_M:
            self.root.after(
                0,
                lambda: self._set_status(
                    f"{algo} is limited to local trips under {LOCAL_ROUTE_MAX_M / 1000:.0f} km.",
                    "error",
                ),
            )
            return None

        margin_steps = [None] if straight_m <= LOCAL_ROUTE_MAX_M else list(LONG_ROUTE_MARGIN_STEPS_M)
        best_result = None

        for margin in margin_steps:
            if margin is None:
                status_text = f"Building local route graph for {algo}..."
            else:
                status_text = f"Building corridor graph ({int(margin/1000)} km margin)..."

            self.root.after(0, lambda text=status_text: self._set_status(text, "info"))

            route_graph = self.loader.load_route_graph(
                orig_lat,
                orig_lon,
                dest_lat,
                dest_lon,
                forced_margin_m=margin,
            )

            if route_graph is None or route_graph.number_of_nodes() == 0:
                continue

            orig_node = self._nearest_node_in_graph(route_graph, orig_lat, orig_lon)
            dest_node = self._nearest_node_in_graph(route_graph, dest_lat, dest_lon)
            if orig_node is None or dest_node is None:
                continue

            t0 = time.time()
            path = find_path(route_graph, orig_node, dest_node, algorithm=algo)
            elapsed_ms = (time.time() - t0) * 1000.0

            if path:
                best_result = (route_graph, path, dest_node, elapsed_ms)
                break

        if best_result is None:
            self.root.after(0, lambda: self._set_status("No route found in the current route graph.", "error"))

        return best_result

    def _check_off_route(self, lat: float, lon: float):
        if self.dest_latlon is None or self._routing_active:
            return

        self._last_reroute_pos = (lat, lon)
        algo = self.route_algorithm

        def _worker():
            try:
                route_result = self._compute_route(
                    lat,
                    lon,
                    self.dest_latlon[0],
                    self.dest_latlon[1],
                    algo,
                )
                if route_result is None:
                    return

                route_graph, path, dest_node, _ = route_result

                def _apply():
                    self.route_nodes = path
                    self.route_graph = route_graph
                    self.dest_node = dest_node
                    self.renderer.set_route_coords(route_graph, path)
                    self._set_status("Re-routed.", "ok")

                self.root.after(0, _apply)
            except Exception:
                logger.exception("Re-route failed")

        threading.Thread(target=_worker, daemon=True).start()

    def _clear_route(self):
        self.route_nodes = []
        self.dest_latlon = None
        self.dest_node = None
        self.route_graph = None
        self.renderer.clear_route_cache()
        self._lbl_dist.config(text="Distance: --")
        self._lbl_eta.config(text="ETA: --")
        self._lbl_algo_used.config(text="Algorithm: --")
        self._set_status("Route cleared.", "info")

    def _path_length(self, path: list, G=None) -> float:
        G = G or self.route_graph or self.loader.G
        if G is None:
            return 0.0

        total = 0.0
        for u, v in zip(path[:-1], path[1:]):
            data = G.get_edge_data(u, v)
            if data:
                lengths = [d.get("length", 0.0) for d in data.values()]
                total += min(lengths)
        return total

    def _nearest_node_in_graph(self, G, lat: float, lon: float) -> Optional[int]:
        if G is None or G.number_of_nodes() == 0:
            return None

        best_node = None
        best_dist2 = float("inf")
        lon_scale = 111_320 * max(math.cos(math.radians(lat)), 0.2)

        for node_id, data in G.nodes(data=True):
            dy = (data["y"] - lat) * 111_320
            dx = (data["x"] - lon) * lon_scale
            dist2 = dx * dx + dy * dy
            if dist2 < best_dist2:
                best_dist2 = dist2
                best_node = node_id

        return best_node

    def _should_refresh_display(self, lat: float, lon: float) -> bool:
        if self._last_display_refresh_pos is None:
            return True

        moved = self._haversine(
            lat,
            lon,
            self._last_display_refresh_pos[0],
            self._last_display_refresh_pos[1],
        )
        return moved >= DISPLAY_REFRESH_MIN_MOVE_M

    def _pan_start(self, event):
        self._pan_start_xy = (event.x, event.y)
        self._pan_start_ll = (self.viewport.center_lat, self.viewport.center_lon)
        self._user_panning = True
        self._pan_time = time.time()

    def _pan_drag(self, event):
        if not hasattr(self, "_pan_start_xy"):
            return

        sx, sy = self._pan_start_xy
        dx = event.x - sx
        dy = event.y - sy

        m_per_deg_lat = 111_320
        m_per_deg_lon = 111_320 * math.cos(math.radians(self.viewport.center_lat))
        scale = self.viewport._base_scale * self.viewport.zoom
        if scale == 0:
            return

        dlat = dy / (m_per_deg_lat * scale)
        dlon = -dx / (m_per_deg_lon * scale)

        start_lat, start_lon = self._pan_start_ll
        self.viewport.center_lat = start_lat + dlat
        self.viewport.center_lon = start_lon + dlon

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

    def _show_keyboard(self, event=None):
        for widget in self.root.winfo_children():
            if isinstance(widget, OSKKeyboard):
                widget.destroy()
                return
        OSKKeyboard(self.root, self._dest_entry)

    def _set_status(self, msg: str, level: str = "info"):
        colors = {
            "ok": ACCENT,
            "info": ACCENT2,
            "warning": "#f0a500",
            "error": DEST_CLR,
        }
        self._status_var.set(msg)
        self._status_lbl.config(fg=colors.get(level, TEXT_MUTED))
        logger.info("[%s] %s", level, msg)

    def _update_scale_bar(self):
        if self.viewport._base_scale == 0:
            return

        m_per_px = 1 / (self.viewport._base_scale * self.viewport.zoom)
        m_100 = m_per_px * 100
        if m_100 >= 1000:
            label = f"<- 100px = {m_100 / 1000:.1f} km"
        else:
            label = f"<- 100px = {m_100:.0f} m"

        self._scale_lbl.config(text=label)

    @staticmethod
    def _haversine(lat1, lon1, lat2, lon2) -> float:
        r = 6_371_000
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = (
            math.sin(dlat / 2) ** 2
            + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        )
        return r * 2 * math.asin(math.sqrt(a))

    def _on_close(self):
        self.gps.stop()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = PiGPSApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
