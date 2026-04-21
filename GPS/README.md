# PiGPS Navigator

A full-featured GPS navigation system for the Raspberry Pi Zero 2W with a 7" 1024×600 HDMI touchscreen and NEO-6M GPS module.

---

## Hardware Setup

### Wiring the NEO-6M GPS Module

| GPS Pin | Pi Zero 2W Pin | GPIO |
|---------|---------------|------|
| VCC     | Pin 1 (3.3V) or Pin 2 (5V) | — |
| GND     | Pin 6 (GND)   | — |
| TX      | Pin 10 (RXD)  | GPIO 15 |
| RX      | Pin 8 (TXD)   | GPIO 14 *(optional)* |

### Enable UART on Pi Zero 2W

1. Edit `/boot/config.txt` — add at the bottom:
   ```
   enable_uart=1
   dtoverlay=disable-bt
   ```

2. Edit `/boot/cmdline.txt` — remove this part if present:
   ```
   console=serial0,115200
   ```

3. Disable serial console service:
   ```bash
   sudo systemctl disable serial-getty@ttyS0.service
   sudo systemctl stop serial-getty@ttyS0.service
   ```

4. Reboot:
   ```bash
   sudo reboot
   ```

5. Test GPS data is coming through:
   ```bash
   sudo cat /dev/ttyS0
   ```
   You should see NMEA sentences like `$GNGGA,...` or `$GNRMC,...`

---

## Software Installation

```bash
# Install Python dependencies
pip install pyserial pynmea2 --break-system-packages

# OSMnx (if not already installed)
pip install osmnx --break-system-packages

# Tkinter (if missing)
sudo apt install python3-tk
```

---

## Project File Structure

```
gps_app/
├── main.py          — Tkinter UI & application controller
├── gps_reader.py    — NEO-6M serial NMEA parser (with simulation fallback)
├── map_engine.py    — OSMnx graph loader & Tkinter canvas renderer
├── routing.py       — Dijkstra, A*, Bellman-Ford, ACO algorithms
├── config.py        — All settings in one place
└── requirements.txt — Python dependencies
```

---

## Running the App

```bash
cd gps_app
python3 main.py
```

> **Note:** On first launch with no GPS fix, the app runs in simulation mode (walking a small loop near Cookeville, TN). Once the NEO-6M has a satellite fix (typically 30–90 seconds outdoors), it switches to live GPS automatically.

---

## How It Works

### GPS Flow
1. `GPSReader` opens `/dev/ttyS0` in a background thread
2. NMEA sentences (`$GNGGA`, `$GNRMC`, `$GNVTG`) are parsed by `pynmea2`
3. Lat/lon, speed, heading, and satellite count are updated every second
4. If `pyserial`/`pynmea2` are not installed, a simulation mode activates automatically

### Map Loading
1. On first GPS fix, `GraphLoader.load()` calls `osmnx.graph_from_point()` centred on your location with a 3 km radius
2. The MultiDiGraph is speed/travel-time annotated and stored in memory
3. OSMnx's built-in HTTP cache (`~/.pigps_cache`) prevents re-downloading the same area

### Routing
1. User types a destination address and selects an algorithm
2. `osmnx.geocode()` converts the address to lat/lon
3. `osmnx.nearest_nodes()` snaps both origin and destination to the road network
4. The selected algorithm runs on the MultiDiGraph and returns an ordered list of OSM node IDs
5. Distance is computed by summing edge `length` attributes; ETA uses current GPS speed (minimum 30 km/h assumed)

### Algorithms

| Algorithm    | Best for | Notes |
|-------------|----------|-------|
| **A\***     | Fastest real-world paths | Uses Haversine heuristic; best default |
| **Dijkstra** | Guaranteed shortest path | Slower than A\* on large graphs |
| **Bellman-Ford** | Graphs with negative weights | Slowest; handles edge cases |
| **ACO**     | Exploratory/heuristic routing | Probabilistic; good for multiple route suggestions |

### Live Tracking & Re-routing
- The map redraws every second with the current GPS position overlaid as a green dot
- If you deviate more than 50 m from the route, the app automatically re-routes using the same algorithm

---

## Configuration (`config.py`)

| Setting | Default | Description |
|---------|---------|-------------|
| `GPS_PORT` | `/dev/ttyS0` | UART port for NEO-6M |
| `GPS_BAUD` | `9600` | NEO-6M default baud rate |
| `GRAPH_RADIUS_M` | `3000` | Map download radius in metres |
| `NETWORK_TYPE` | `drive` | Road network type (`drive`/`walk`/`bike`) |
| `REROUTE_DIST_M` | `50` | Off-route threshold for auto re-route |
| `ACO_ANTS` | `30` | Number of ants for ACO |
| `ACO_ITERATIONS` | `50` | ACO iteration count |

---

## Touchscreen Tips

- **Pan**: Drag on the map
- **Zoom**: Mouse scroll wheel, or use the `+` / `−` buttons in the sidebar
- **Re-centre**: Press `⊙ Re-centre` to snap the view back to your GPS position
- **Load Map**: Press `⬇ Load Map` to download a new map around your current position (useful after driving far)

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `No GPS fix` | Ensure antenna is outdoors or near a window; wait 60–90 s |
| `serial.SerialException` | Check wiring; run `ls /dev/ttyS*`; verify UART is enabled |
| `Geocode failed` | Needs internet; ensure Pi has WiFi/Ethernet |
| `Map load error` | Check internet; try increasing `GRAPH_RADIUS_M` in `config.py` |
| App runs slow | Lower `GRAPH_RADIUS_M` to `1500`; ACO is the slowest algorithm on large graphs |
| Touchscreen not registering | Check HDMI display config in `/boot/config.txt` for your display model |

---

## Memory & Storage

- A 3 km radius OSMnx graph typically uses ~30–60 MB RAM on Pi Zero 2W
- Graph HTTP cache grows over time in `~/.pigps_cache`; clear it with `rm -rf ~/.pigps_cache` if needed
- The 20 GB free on your SD card is more than sufficient
