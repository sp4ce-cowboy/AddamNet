"""
main.py
─────────────────────────────────────────────────────────────────
FastAPI application entry point.  Creates and wires all services,
starts background threads, and serves the dashboard.
"""

import os
import sys
import time
import threading
import logging
from pathlib import Path
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse

# ── Ensure project root is on sys.path ──
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from app import config as cfg
from app.services.sensor_service import SensorService
from app.services.video_service import VideoService
from app.services.yolo_service import YOLOService
from app.services.defect_logger import DefectLogger
from app.services.slam_service import SLAMService
from app.services.slam_live_service import SLAMLiveService
from app.api.routes import router, inject_services

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
)
logger = logging.getLogger("main")

# ── Global service instances ──
sensor_svc: SensorService = None
video_svc: VideoService = None
yolo_svc: YOLOService = None
defect_log: DefectLogger = None
slam_svc: SLAMService = None
slam_live_svc: SLAMLiveService = None
_inference_thread: threading.Thread = None
_inference_running = False


def init_services():
    """Instantiate all services from config."""
    global sensor_svc, video_svc, yolo_svc, defect_log, slam_svc, slam_live_svc

    c = cfg.get()

    # Sensor service
    ser_cfg = c.get("serial", {})
    sensor_svc = SensorService(
        port=ser_cfg.get("port", "auto"),
        baud=ser_cfg.get("baud_rate", 115200),
        timeout=ser_cfg.get("timeout", 2),
        mock_fallback=ser_cfg.get("mock_fallback", True),
    )

    # Video service
    vid_cfg = c.get("video", {})
    video_svc = VideoService(
        source=vid_cfg.get("source", "webcam"),
        webcam_device=vid_cfg.get("webcam_device", 0),
        video_file=cfg.resolve(vid_cfg.get("video_file", "")),
        target_fps=vid_cfg.get("target_fps", 5),
        frame_width=vid_cfg.get("frame_width", 640),
        frame_height=vid_cfg.get("frame_height", 480),
    )

    # YOLO service
    yolo_cfg = c.get("yolo", {})
    yolo_svc = YOLOService(
        model_path=cfg.resolve(yolo_cfg.get("model_path", "weights/best.pt")),
        confidence_threshold=yolo_cfg.get("confidence_threshold", 0.35),
        imgsz=yolo_cfg.get("imgsz", 320),
        class_names=yolo_cfg.get("class_names", {}),
    )

    # Defect logger
    def_cfg = c.get("defects", {})
    defect_log = DefectLogger(
        db_path=cfg.resolve(def_cfg.get("db_path", "data/defects.db")),
        cooldown_seconds=def_cfg.get("cooldown_seconds", 5.0),
    )

    # SLAM service (offline batch)
    slam_cfg = c.get("slam", {})
    slam_svc = SLAMService(
        output_dir=cfg.resolve(slam_cfg.get("output_dir", "data/slam_output")),
    )

    # Live SLAM service (real-time visualization)
    slam_cfg = c.get("slam", {})
    if slam_cfg.get("live_enabled", True):
        slam_live_svc = SLAMLiveService(
            slam_module_dir=str(cfg.project_root() / "slam"),
            process_every_n=slam_cfg.get("live_process_every_n", 2),
        )
    else:
        slam_live_svc = None

    # Wire services into routes
    inject_services(sensor_svc, video_svc, yolo_svc, defect_log, slam_svc, slam_live_svc)


def start_services():
    """Start background threads for sensor reading, video capture, inference."""
    global _inference_running, _inference_thread

    sensor_svc.start()
    video_svc.start()

    # Load YOLO model (may take a few seconds on RPi)
    logger.info("Loading YOLO model...")
    yolo_svc.load_model()

    # Start the inference loop in a background thread
    _inference_running = True
    _inference_thread = threading.Thread(target=_inference_loop, daemon=True)
    _inference_thread.start()

    logger.info("All services started")


def stop_services():
    global _inference_running
    _inference_running = False
    sensor_svc.stop()
    video_svc.stop()
    logger.info("All services stopped")


def _inference_loop():
    """
    Background loop: pull frames from video service, run YOLO,
    log defects.  Paced by the video service's target FPS.
    """
    c = cfg.get()
    target_fps = c.get("video", {}).get("target_fps", 5)
    interval = 1.0 / max(target_fps, 1)

    logger.info(f"Inference loop started (target {target_fps} FPS)")

    while _inference_running:
        t0 = time.time()

        frame, frame_num = video_svc.get_frame()
        if frame is not None:
            # YOLO inference
            if yolo_svc.loaded:
                detections = yolo_svc.run_inference(frame, frame_num)

                # Log defect events
                if detections and defect_log:
                    defect_log.log_detections(
                        detections=detections,
                        frame_number=frame_num,
                        video_source=video_svc.source_label,
                        sensor_snap=sensor_svc.latest,
                    )

            # Live SLAM visualization (skip logic is internal)
            if slam_live_svc and slam_live_svc.available:
                slam_live_svc.process_frame(frame)

        elapsed = time.time() - t0
        sleep_time = interval - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


# ── FastAPI lifecycle ──

@asynccontextmanager
async def lifespan(app: FastAPI):
    init_services()
    start_services()
    yield
    stop_services()


# ── Create app ──
app = FastAPI(
    title="Pipeline Inspection Dashboard",
    version="1.0.0",
    lifespan=lifespan,
)

# Mount static files (CSS / JS / images)
static_dir = Path(__file__).parent / "static"
app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")

# Include API routes
app.include_router(router)


# Serve index.html at root
@app.get("/")
async def serve_dashboard():
    index_path = static_dir / "index.html"
    return FileResponse(str(index_path))
