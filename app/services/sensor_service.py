"""
sensor_service.py
─────────────────────────────────────────────────────────────────
Background thread that reads JSON sensor data from the Arduino
Mega over USB serial, parses it, and maintains the latest sensor
state in memory.  Adapted from the user's serial_reader.py.

Falls back to mock data generation when serial is unavailable.
"""

import json
import time
import random
import threading
import logging
from datetime import datetime
from typing import Optional, Callable

from app.models.schemas import SensorSnapshot

logger = logging.getLogger("sensor_service")

# ── Try importing pyserial (not available in all envs) ──
try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False
    logger.warning("pyserial not installed — serial features disabled")


def auto_detect_port() -> Optional[str]:
    """Return the first serial port that looks like an Arduino."""
    if not HAS_SERIAL:
        return None
    candidates = []
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        mfr = (p.manufacturer or "").lower()
        if any(k in desc or k in mfr for k in ("arduino", "ch340", "cp210", "ftdi", "acm")):
            candidates.append(p.device)
    if not candidates:
        for p in serial.tools.list_ports.comports():
            if "ttyACM" in p.device or "ttyUSB" in p.device:
                candidates.append(p.device)
    return candidates[0] if candidates else None


def parse_sensor_json(data: dict) -> SensorSnapshot:
    """Convert raw Arduino JSON dict into a flat SensorSnapshot."""
    snap = SensorSnapshot(timestamp=datetime.utcnow(), raw_json=data)
    snap.uptime_s = data.get("uptime_s")
    snap.master_enabled = data.get("master_enabled")

    dht = data.get("dht22", {})
    snap.temperature_c = dht.get("temperature_c")
    snap.humidity_pct = dht.get("humidity_pct")

    dist = data.get("distance", {})
    snap.ultrasonic_cm = dist.get("ultrasonic_cm")
    snap.sharp_ir_cm = dist.get("sharp_ir_cm")
    snap.average_distance_cm = dist.get("average_cm")
    hw = dist.get("hw201_ir", {})
    snap.obstacle_detected = hw.get("obstacle_detected")

    mpu = data.get("mpu6050", {})
    vel = mpu.get("velocity_ms", {})
    snap.speed_ms = vel.get("speed")
    snap.total_distance_m = mpu.get("total_distance_m")
    accel = mpu.get("accel_ms2", {})
    snap.accel_x_ms2 = accel.get("x")
    snap.accel_y_ms2 = accel.get("y")
    snap.accel_z_ms2 = accel.get("z")
    orient = mpu.get("orientation_deg", {})
    snap.roll_deg = orient.get("roll")
    snap.pitch_deg = orient.get("pitch")
    snap.yaw_deg = orient.get("yaw")
    snap.tether_exceeded = mpu.get("tether_exceeded")

    mq = data.get("mq135", {})
    snap.air_quality_raw = mq.get("raw_adc")
    snap.air_quality_label = mq.get("quality")

    return snap


class SensorService:
    """
    Manages serial connection and sensor state.
    Runs a background thread that continuously reads from serial
    (or generates mock data).
    """

    def __init__(self, port="auto", baud=115200, timeout=2, mock_fallback=True):
        self._port_cfg = port
        self._baud = baud
        self._timeout = timeout
        self._mock_fallback = mock_fallback

        self._serial: Optional[object] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._connected = False
        self._actual_port: Optional[str] = None

        # Latest sensor snapshot (thread-safe via GIL for simple reads)
        self.latest: Optional[SensorSnapshot] = None
        # Optional callback for each new reading
        self.on_update: Optional[Callable] = None

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def port(self) -> Optional[str]:
        return self._actual_port

    def start(self):
        """Start the background reader thread."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        logger.info("Sensor service started")

    def stop(self):
        self._running = False
        if self._serial:
            try:
                self._serial.close()
            except Exception:
                pass
        logger.info("Sensor service stopped")

    # ── Internal ──

    def _connect_serial(self) -> bool:
        if not HAS_SERIAL:
            return False
        port = self._port_cfg
        if port == "auto":
            port = auto_detect_port()
        if port is None or port == "mock":
            return False
        try:
            self._serial = serial.Serial(port, self._baud, timeout=self._timeout)
            self._actual_port = port
            self._connected = True
            logger.info(f"Serial connected: {port} @ {self._baud}")
            return True
        except Exception as e:
            logger.warning(f"Serial connection failed ({port}): {e}")
            return False

    def _run_loop(self):
        """Main reader loop — tries serial, falls back to mock."""
        connected = self._connect_serial()
        if not connected and self._mock_fallback:
            logger.info("Using MOCK sensor data")
            self._run_mock_loop()
            return
        if not connected:
            logger.error("No serial port and mock disabled — sensor thread exiting")
            return
        self._run_serial_loop()

    def _run_serial_loop(self):
        while self._running:
            try:
                raw = self._serial.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                try:
                    data = json.loads(line)
                except json.JSONDecodeError:
                    continue
                # Skip status/init messages (no uptime_s)
                if "uptime_s" not in data:
                    continue
                snap = parse_sensor_json(data)
                self.latest = snap
                if self.on_update:
                    self.on_update(snap)
            except Exception as e:
                logger.error(f"Serial read error: {e}")
                self._connected = False
                time.sleep(2)
                # Try to reconnect
                if self._connect_serial():
                    continue
                elif self._mock_fallback:
                    logger.info("Switching to MOCK data after serial loss")
                    self._run_mock_loop()
                    return
                else:
                    break

    def _run_mock_loop(self):
        """Generate plausible fake sensor data at ~1 Hz."""
        self._actual_port = "mock"
        t0 = time.time()
        while self._running:
            elapsed = time.time() - t0
            snap = SensorSnapshot(
                timestamp=datetime.utcnow(),
                uptime_s=round(elapsed, 2),
                temperature_c=round(24.0 + random.uniform(-0.5, 0.5), 1),
                humidity_pct=round(60.0 + random.uniform(-2, 2), 1),
                ultrasonic_cm=round(random.uniform(4, 15), 1),
                sharp_ir_cm=round(random.uniform(100, 150), 1),
                average_distance_cm=round(random.uniform(50, 80), 1),
                obstacle_detected=random.random() < 0.05,
                air_quality_raw=random.randint(45, 75),
                air_quality_label="Normal",
                speed_ms=round(random.uniform(0, 3.5), 1),
                total_distance_m=round(elapsed * 2.5, 1),
                accel_x_ms2=round(random.uniform(-0.3, 0.3), 2),
                accel_y_ms2=round(random.uniform(-0.3, 0.3), 2),
                accel_z_ms2=round(random.uniform(-0.1, 0.1), 2),
                roll_deg=round(random.uniform(-5, 5), 2),
                pitch_deg=round(random.uniform(-5, 5), 2),
                yaw_deg=round(random.uniform(0, 360), 2),
                tether_exceeded=False,
                master_enabled=True,
            )
            self.latest = snap
            if self.on_update:
                self.on_update(snap)
            time.sleep(1.0)
