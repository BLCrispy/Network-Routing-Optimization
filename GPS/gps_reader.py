# gps_reader.py — NEO-6M UART reader using pynmea2
#
# Wiring (Pi Zero 2W):
#   GPS TX  → Pi GPIO 15 (RXD / pin 10)
#   GPS RX  → Pi GPIO 14 (TXD / pin 8)   [optional, for commands]
#   GPS VCC → Pi 3.3 V or 5 V (module accepts 3–5 V)
#   GPS GND → Pi GND
#
# Enable UART on Pi Zero 2W:
#   In /boot/config.txt add:  enable_uart=1
#   In /boot/cmdline.txt remove: console=serial0,115200
#   Then: sudo systemctl disable serial-getty@ttyS0.service

import threading
import time
import logging

try:
    import serial
    import pynmea2
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

from config import GPS_PORT, GPS_BAUD, GPS_TIMEOUT

logger = logging.getLogger(__name__)


class GPSReader:
    """
    Background thread that continuously reads NMEA sentences from the NEO-6M
    and exposes the latest fix via thread-safe properties.
    """

    def __init__(self):
        self._lat        = None
        self._lon        = None
        self._speed_kmh  = 0.0
        self._heading    = 0.0
        self._fix        = False
        self._satellites = 0
        self._lock       = threading.Lock()
        self._running    = False
        self._thread     = None
        self._ser        = None
        self._sim_mode   = not SERIAL_AVAILABLE  # fall back to simulation

    # ── Public API ────────────────────────────────────────────────────────────

    def start(self):
        self._running = True
        if self._sim_mode:
            logger.warning("serial/pynmea2 not found — running GPS simulation mode")
            self._thread = threading.Thread(target=self._simulate, daemon=True)
        else:
            self._thread = threading.Thread(target=self._read_serial, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        time.sleep(0.2)
        try:
            if self._ser and self._ser.is_open:
                self._ser.flush()
                self._ser.close()
                self._ser = None
        except Exception as e:
            logger.warning(f"GPS port close error: {e}")

    @property
    def position(self):
        """Returns (lat, lon) or (None, None) if no fix."""
        with self._lock:
            return self._lat, self._lon

    @property
    def has_fix(self):
        with self._lock:
            return self._fix

    @property
    def speed_kmh(self):
        with self._lock:
            return self._speed_kmh

    @property
    def heading(self):
        with self._lock:
            return self._heading

    @property
    def satellites(self):
        with self._lock:
            return self._satellites

    # ── Internal ──────────────────────────────────────────────────────────────

    def _read_serial(self):
        try:
            self._ser = serial.Serial(GPS_PORT, GPS_BAUD, timeout=GPS_TIMEOUT)
            logger.info(f"Opened GPS on {GPS_PORT} @ {GPS_BAUD} baud")
        except Exception as e:
            logger.error(f"Cannot open GPS serial port: {e}")
            self._sim_mode = True
            self._simulate()
            return

        try:
            while self._running:
                try:
                    line = self._ser.readline().decode("ascii", errors="replace").strip()
                    if line.startswith("$"):
                        self._parse_nmea(line)
                except Exception as e:
                    logger.debug(f"GPS read error: {e}")
                    time.sleep(0.5)
        finally:
            try:
                if self._ser and self._ser.is_open:
                    self._ser.close()
                    self._ser = None
                    logger.info("GPS serial port closed cleanly")
            except Exception:
                pass

    def _parse_nmea(self, sentence: str):
        try:
            msg = pynmea2.parse(sentence)
        except pynmea2.ParseError:
            return

        if isinstance(msg, pynmea2.types.talker.GGA):
            with self._lock:
                self._fix        = int(getattr(msg, "gps_qual", 0)) > 0
                self._satellites = int(getattr(msg, "num_sats", 0) or 0)
                if self._fix and msg.latitude and msg.longitude:
                    self._lat = msg.latitude
                    self._lon = msg.longitude

        elif isinstance(msg, pynmea2.types.talker.RMC):
            with self._lock:
                if msg.status == "A" and msg.latitude and msg.longitude:
                    self._fix      = True
                    self._lat      = msg.latitude
                    self._lon      = msg.longitude
                    spd = getattr(msg, "spd_over_grnd", 0)
                    self._speed_kmh = float(spd or 0) * 1.852
                    hdg = getattr(msg, "true_course", 0)
                    self._heading  = float(hdg or 0)

        elif isinstance(msg, pynmea2.types.talker.VTG):
            with self._lock:
                spd = getattr(msg, "spd_over_grnd_kmph", 0)
                self._speed_kmh = float(spd or 0)

    # ── Simulation (no hardware) ──────────────────────────────────────────────

    def _simulate(self):
        """
        Walks a tiny route so you can develop without hardware.
        Starts near Cookeville, TN city centre.
        """
        import math
        base_lat, base_lon = 36.1628, -85.5016
        step = 0
        while self._running:
            lat = base_lat + math.sin(step * 0.05) * 0.002
            lon = base_lon + math.cos(step * 0.05) * 0.002
            with self._lock:
                self._lat        = lat
                self._lon        = lon
                self._fix        = True
                self._speed_kmh  = 30.0
                self._heading    = (step * 5) % 360
                self._satellites = 8
            step += 1
            time.sleep(1)
