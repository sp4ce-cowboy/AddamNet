"""
defect_logger.py
─────────────────────────────────────────────────────────────────
Maintains a SQLite database of defect events.  Each event is a
snapshot created when YOLO detects a defect, enriched with the
nearest sensor reading at that moment.

Implements a per-class cooldown / debounce so repeated detections
of the same class over a short window do not spam the log.
"""

import sqlite3
import time
import threading
import logging
from datetime import datetime
from typing import Optional, List, Dict, Any

from app.models.schemas import DefectEvent, SensorSnapshot

logger = logging.getLogger("defect_logger")

CREATE_TABLE_SQL = """
CREATE TABLE IF NOT EXISTS defects (
    id              INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp       TEXT    NOT NULL,
    defect_class    TEXT    NOT NULL,
    confidence      REAL    NOT NULL,
    frame_number    INTEGER NOT NULL,
    video_source    TEXT    NOT NULL,
    sensor_temperature_c    REAL,
    sensor_humidity_pct     REAL,
    sensor_ultrasonic_cm    REAL,
    sensor_sharp_ir_cm      REAL,
    sensor_avg_distance_cm  REAL,
    sensor_speed_ms         REAL,
    sensor_total_distance_m REAL,
    sensor_air_quality      TEXT
);
"""


class DefectLogger:
    """
    Thread-safe defect event logger with cooldown deduplication.
    """

    def __init__(self, db_path: str, cooldown_seconds: float = 5.0):
        self._db_path = db_path
        self._cooldown = cooldown_seconds
        self._lock = threading.Lock()
        self._active = False

        # Per-class last-logged timestamps for cooldown
        self._last_logged: Dict[str, float] = {}

        self._init_db()

    def _init_db(self):
        try:
            conn = sqlite3.connect(self._db_path)
            conn.execute(CREATE_TABLE_SQL)
            conn.commit()
            conn.close()
            self._active = True
            logger.info(f"Defect DB ready: {self._db_path}")
        except Exception as e:
            logger.error(f"DB init failed: {e}")

    @property
    def active(self) -> bool:
        return self._active

    def log_detections(
        self,
        detections: List[Dict[str, Any]],
        frame_number: int,
        video_source: str,
        sensor_snap: Optional[SensorSnapshot] = None,
    ) -> List[DefectEvent]:
        """
        Log detections (with cooldown filtering).
        Returns list of events that were actually saved.
        """
        saved = []
        now = time.time()

        for det in detections:
            cls_name = det["class_name"]
            conf = det["confidence"]

            # ── Cooldown check ──
            last = self._last_logged.get(cls_name, 0)
            if now - last < self._cooldown:
                continue
            self._last_logged[cls_name] = now

            ts = datetime.utcnow().isoformat(timespec="seconds") + "Z"

            event = DefectEvent(
                timestamp=ts,
                defect_class=cls_name,
                confidence=conf,
                frame_number=frame_number,
                video_source=video_source,
            )

            # Attach sensor data if available
            if sensor_snap:
                event.sensor_temperature_c = sensor_snap.temperature_c
                event.sensor_humidity_pct = sensor_snap.humidity_pct
                event.sensor_ultrasonic_cm = sensor_snap.ultrasonic_cm
                event.sensor_sharp_ir_cm = sensor_snap.sharp_ir_cm
                event.sensor_avg_distance_cm = sensor_snap.average_distance_cm
                event.sensor_speed_ms = sensor_snap.speed_ms
                event.sensor_total_distance_m = sensor_snap.total_distance_m
                event.sensor_air_quality = sensor_snap.air_quality_label

            self._insert(event)
            saved.append(event)
            logger.info(f"Defect logged: {cls_name} ({conf:.2f}) frame={frame_number}")

        return saved

    def _insert(self, ev: DefectEvent):
        with self._lock:
            try:
                conn = sqlite3.connect(self._db_path)
                conn.execute(
                    """INSERT INTO defects
                       (timestamp, defect_class, confidence, frame_number,
                        video_source, sensor_temperature_c, sensor_humidity_pct,
                        sensor_ultrasonic_cm, sensor_sharp_ir_cm,
                        sensor_avg_distance_cm, sensor_speed_ms,
                        sensor_total_distance_m, sensor_air_quality)
                       VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)""",
                    (
                        ev.timestamp, ev.defect_class, ev.confidence,
                        ev.frame_number, ev.video_source,
                        ev.sensor_temperature_c, ev.sensor_humidity_pct,
                        ev.sensor_ultrasonic_cm, ev.sensor_sharp_ir_cm,
                        ev.sensor_avg_distance_cm, ev.sensor_speed_ms,
                        ev.sensor_total_distance_m, ev.sensor_air_quality,
                    ),
                )
                conn.commit()
                conn.close()
            except Exception as e:
                logger.error(f"DB insert failed: {e}")

    def get_all(self, limit: int = 200) -> List[DefectEvent]:
        """Return recent defect events, newest first."""
        with self._lock:
            try:
                conn = sqlite3.connect(self._db_path)
                conn.row_factory = sqlite3.Row
                rows = conn.execute(
                    "SELECT * FROM defects ORDER BY id DESC LIMIT ?", (limit,)
                ).fetchall()
                conn.close()
                return [DefectEvent(**dict(r)) for r in rows]
            except Exception as e:
                logger.error(f"DB query failed: {e}")
                return []

    def get_count(self) -> int:
        with self._lock:
            try:
                conn = sqlite3.connect(self._db_path)
                count = conn.execute("SELECT COUNT(*) FROM defects").fetchone()[0]
                conn.close()
                return count
            except Exception:
                return 0

    def delete_by_id(self, defect_id: int) -> bool:
        """Delete a single defect event by its ID. Returns True if a row was deleted."""
        with self._lock:
            try:
                conn = sqlite3.connect(self._db_path)
                cur = conn.execute("DELETE FROM defects WHERE id = ?", (defect_id,))
                conn.commit()
                deleted = cur.rowcount > 0
                conn.close()
                return deleted
            except Exception as e:
                logger.error(f"DB delete failed: {e}")
                return False

    def clear(self):
        """Delete all defect records (for testing)."""
        with self._lock:
            try:
                conn = sqlite3.connect(self._db_path)
                conn.execute("DELETE FROM defects")
                conn.commit()
                conn.close()
                self._last_logged.clear()
            except Exception as e:
                logger.error(f"DB clear failed: {e}")
