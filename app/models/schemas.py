"""
schemas.py — Pydantic models shared across the application.
"""

from pydantic import BaseModel
from typing import Optional, Dict, Any, List
from datetime import datetime


class SensorSnapshot(BaseModel):
    """Latest parsed sensor reading from Arduino."""
    timestamp: datetime = datetime.utcnow()
    uptime_s: Optional[float] = None
    temperature_c: Optional[float] = None
    humidity_pct: Optional[float] = None
    ultrasonic_cm: Optional[float] = None
    sharp_ir_cm: Optional[float] = None
    average_distance_cm: Optional[float] = None
    obstacle_detected: Optional[bool] = None
    air_quality_raw: Optional[int] = None
    air_quality_label: Optional[str] = None
    speed_ms: Optional[float] = None
    total_distance_m: Optional[float] = None
    accel_x_ms2: Optional[float] = None
    accel_y_ms2: Optional[float] = None
    accel_z_ms2: Optional[float] = None
    roll_deg: Optional[float] = None
    pitch_deg: Optional[float] = None
    yaw_deg: Optional[float] = None
    tether_exceeded: Optional[bool] = None
    master_enabled: Optional[bool] = None
    raw_json: Optional[Dict[str, Any]] = None


class DefectEvent(BaseModel):
    """A single logged defect detection event."""
    id: Optional[int] = None
    timestamp: str
    defect_class: str
    confidence: float
    frame_number: int
    video_source: str
    # sensor snapshot at detection time
    sensor_temperature_c: Optional[float] = None
    sensor_humidity_pct: Optional[float] = None
    sensor_ultrasonic_cm: Optional[float] = None
    sensor_sharp_ir_cm: Optional[float] = None
    sensor_avg_distance_cm: Optional[float] = None
    sensor_speed_ms: Optional[float] = None
    sensor_total_distance_m: Optional[float] = None
    sensor_air_quality: Optional[str] = None


class SystemStatus(BaseModel):
    serial_connected: bool = False
    serial_port: Optional[str] = None
    camera_active: bool = False
    video_source: str = "none"
    yolo_loaded: bool = False
    slam_available: bool = False
    db_active: bool = False


class SLAMStatus(BaseModel):
    running: bool = False
    progress_pct: float = 0.0
    last_run_time: Optional[str] = None
    viewer_html_path: Optional[str] = None
    output_files: List[str] = []
    message: str = "Idle"
