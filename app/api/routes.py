"""
routes.py
─────────────────────────────────────────────────────────────────
All FastAPI routes for the pipeline inspection dashboard.

Endpoints:
  GET  /api/sensors          → latest sensor snapshot
  GET  /api/defects          → list of defect events
  GET  /api/status           → system status summary
  GET  /api/slam             → SLAM status
  POST /api/slam/run         → trigger a SLAM run
  GET  /api/feed/frame       → latest annotated JPEG frame
  GET  /api/detections       → latest YOLO detections list
  WS   /ws/live              → WebSocket for real-time updates
  GET  /slam-viewer          → serve generated SLAM HTML viewer
"""

import asyncio
import json
import os
import time
import logging
from typing import Optional

from fastapi import APIRouter, WebSocket, WebSocketDisconnect, Query, UploadFile, File
from fastapi.responses import Response, HTMLResponse, JSONResponse
from starlette.websockets import WebSocketState

logger = logging.getLogger("routes")

router = APIRouter()

# ── These get injected by main.py at startup ──
sensor_service = None
video_service = None
yolo_service = None
defect_logger = None
slam_service = None
live_slam_service = None


def inject_services(sensor, video, yolo, defects, slam, live_slam=None):
    """Called by main.py to wire up services."""
    global sensor_service, video_service, yolo_service, defect_logger, slam_service, live_slam_service
    sensor_service = sensor
    video_service = video
    yolo_service = yolo
    defect_logger = defects
    slam_service = slam
    live_slam_service = live_slam


# ─────────────────────────────────────────────────────────────────
#  REST Endpoints
# ─────────────────────────────────────────────────────────────────

@router.get("/api/sensors")
async def get_sensors():
    """Return latest sensor snapshot."""
    if sensor_service and sensor_service.latest:
        snap = sensor_service.latest
        return snap.model_dump(exclude={"raw_json"})
    return {"message": "No sensor data yet"}


@router.get("/api/defects")
async def get_defects(limit: int = Query(200, ge=1, le=1000)):
    """Return recent defect events."""
    if defect_logger:
        events = defect_logger.get_all(limit=limit)
        return [e.model_dump() for e in events]
    return []


@router.delete("/api/defects/{defect_id}")
async def delete_defect(defect_id: int):
    """Delete a single defect event by ID."""
    if not defect_logger:
        return JSONResponse({"error": "Defect logger not available"}, status_code=503)
    ok = defect_logger.delete_by_id(defect_id)
    if ok:
        return {"message": "Deleted", "id": defect_id}
    return JSONResponse({"error": "Defect not found"}, status_code=404)

@router.delete("/api/defects")
async def clear_all_defects():
    """Delete all defect events."""
    if not defect_logger:
        return JSONResponse({"error": "Defect logger not available"}, status_code=503)
    defect_logger.clear()
    return {"message": "All defects cleared"}

@router.get("/api/status")
async def get_status():
    """Return system-wide status summary."""
    return {
        "serial_connected": sensor_service.connected if sensor_service else False,
        "serial_port": sensor_service.port if sensor_service else None,
        "camera_active": video_service.active if video_service else False,
        "video_source": video_service.source_label if video_service else "none",
        "yolo_loaded": yolo_service.loaded if yolo_service else False,
        "slam_available": slam_service.available if slam_service else False,
        "db_active": defect_logger.active if defect_logger else False,
        "defect_count": defect_logger.get_count() if defect_logger else 0,
        "inference_ms": yolo_service.get_inference_ms() if yolo_service else 0,
    }


@router.get("/api/slam")
async def get_slam_status():
    """Return SLAM pipeline status plus current source info."""
    result = {}
    if slam_service:
        result = slam_service.get_status().model_dump()
    # Add source context so the frontend knows what buttons to show
    if video_service:
        result["current_source"] = video_service._source_type
        result["current_video_file"] = video_service.video_file_path or ""
    if live_slam_service:
        result["live_slam_has_data"] = live_slam_service.has_data()
        result["live_slam_stats"] = live_slam_service.get_stats()
    else:
        result["live_slam_has_data"] = False
    return result


@router.post("/api/slam/run")
async def trigger_slam_run(video_path: str = Query(..., description="Path to video file")):
    """Trigger a SLAM pipeline run on a video file."""
    if not slam_service:
        return JSONResponse({"error": "SLAM service not available"}, status_code=503)
    ok = slam_service.start_run(video_path)
    if ok:
        return {"message": "SLAM run started", "video": video_path}
    return JSONResponse({"error": "Could not start SLAM (already running or bad path)"}, status_code=400)


@router.post("/api/slam/run-current")
async def trigger_slam_on_current_video():
    """Run batch SLAM on the video file currently loaded in the Live Feed."""
    if not slam_service:
        return JSONResponse({"error": "SLAM service not available"}, status_code=503)
    if not video_service or not video_service.video_file_path:
        return JSONResponse({"error": "No video file configured"}, status_code=400)
    video_path = video_service.video_file_path
    ok = slam_service.start_run(video_path)
    if ok:
        return {"message": "SLAM run started on current video", "video": video_path}
    return JSONResponse({"error": "Could not start SLAM (already running or bad path)"}, status_code=400)


@router.post("/api/slam/export-live")
async def export_live_slam():
    """Export the current live SLAM state to PLY, CSV, and HTML viewer."""
    if not live_slam_service:
        return JSONResponse({"error": "Live SLAM service not available"}, status_code=503)
    if not live_slam_service.has_data():
        return JSONResponse({"error": "Not enough SLAM data yet — let it run longer"}, status_code=400)
    from app.config import resolve
    output_dir = resolve("data/slam_output")
    result = live_slam_service.export_results(output_dir)
    if result.get("success"):
        # Also update the offline slam_service so the viewer link works
        if slam_service and result.get("viewer_html"):
            with slam_service._lock:
                slam_service._status.viewer_html_path = result["viewer_html"]
                slam_service._status.output_files = result.get("output_files", [])
                slam_service._status.message = "Live SLAM exported"
                from datetime import datetime
                slam_service._status.last_run_time = datetime.utcnow().isoformat() + "Z"
        return result
    return JSONResponse(result, status_code=400)


@router.get("/api/feed/frame")
async def get_feed_frame():
    """Return latest annotated JPEG frame from YOLO inference."""
    jpg = None
    if yolo_service:
        jpg = yolo_service.get_annotated_jpeg(quality=55)
    if jpg is None and video_service:
        jpg = video_service.get_jpeg(quality=55)
    if jpg:
        return Response(content=jpg, media_type="image/jpeg")
    return Response(content=b"", status_code=204)


@router.get("/api/feed/slam_frame")
async def get_slam_frame():
    """Return latest SLAM visualization JPEG frame."""
    if live_slam_service:
        jpg = live_slam_service.get_viz_jpeg(quality=55)
        if jpg:
            return Response(content=jpg, media_type="image/jpeg")
    return Response(content=b"", status_code=204)


@router.get("/api/detections")
async def get_latest_detections():
    """Return current frame detections (not logged events)."""
    if yolo_service:
        return yolo_service.get_latest_detections()
    return []


@router.post("/api/feed/pause")
async def feed_pause():
    if video_service:
        video_service.pause()
        return {"paused": True}
    return JSONResponse({"error": "No video service"}, status_code=503)


@router.post("/api/feed/resume")
async def feed_resume():
    if video_service:
        video_service.resume()
        return {"paused": False}
    return JSONResponse({"error": "No video service"}, status_code=503)


@router.post("/api/feed/toggle")
async def feed_toggle():
    if video_service:
        paused = video_service.toggle_pause()
        return {"paused": paused}
    return JSONResponse({"error": "No video service"}, status_code=503)


@router.post("/api/feed/seek")
async def feed_seek(fraction: float = Query(..., ge=0.0, le=1.0)):
    """Seek to a position (0.0 = start, 1.0 = end)."""
    if video_service:
        video_service.seek_fraction(fraction)
        return {"seeked": fraction}
    return JSONResponse({"error": "No video service"}, status_code=503)


@router.post("/api/feed/source")
async def feed_source(source: str = Query(..., description="'webcam' or 'file'")):
    """Switch between webcam and video file source."""
    if video_service:
        new = video_service.switch_source(source)
        return {"source": new}
    return JSONResponse({"error": "No video service"}, status_code=503)


@router.post("/api/feed/upload-video")
async def upload_video(file: UploadFile = File(...)):
    """Upload a video file from the browser and switch to it."""
    if not video_service:
        return JSONResponse({"error": "No video service"}, status_code=503)
    from app.config import resolve
    upload_dir = resolve("data/videos")
    os.makedirs(upload_dir, exist_ok=True)
    # Sanitize filename
    safe_name = file.filename.replace("/", "_").replace("\\", "_")
    dest = os.path.join(upload_dir, safe_name)
    # Save uploaded file
    with open(dest, "wb") as f:
        while chunk := await file.read(1024 * 256):
            f.write(chunk)
    # Switch video service to this file
    video_service.set_video_file(dest)
    logger.info(f"Video uploaded and switched: {dest}")
    return {"message": "Video uploaded", "filename": safe_name, "path": dest}


@router.get("/slam-viewer")
async def slam_viewer():
    """Serve the generated SLAM 3D HTML viewer."""
    if slam_service:
        path = slam_service.get_viewer_html_path()
        if path:
            try:
                with open(path, "r") as f:
                    html = f.read()
                return HTMLResponse(content=html)
            except Exception as e:
                return HTMLResponse(content=f"<h3>Error reading viewer: {e}</h3>", status_code=500)
    return HTMLResponse(
        content="<h3>No SLAM viewer available yet. Run a SLAM pipeline first.</h3>",
        status_code=404,
    )


# ─────────────────────────────────────────────────────────────────
#  WebSocket — Real-time updates to frontend
# ─────────────────────────────────────────────────────────────────

@router.websocket("/ws/live")
async def websocket_live(ws: WebSocket):
    """
    Push real-time updates to the dashboard:
      - sensor data (~1 Hz)
      - annotated frame as base64 JPEG (~target FPS)
      - latest detections
    """
    await ws.accept()
    logger.info("WebSocket client connected")

    try:
        while True:
            payload = {}

            # Sensors
            if sensor_service and sensor_service.latest:
                snap = sensor_service.latest
                payload["sensors"] = snap.model_dump(
                    exclude={"raw_json"},
                    mode="json",
                )

            # Annotated frame
            if yolo_service:
                b64 = yolo_service.get_annotated_base64(quality=45)
                if b64:
                    payload["frame_b64"] = b64
                payload["detections"] = yolo_service.get_latest_detections()
                payload["inference_ms"] = yolo_service.get_inference_ms()
            elif video_service:
                b64 = video_service.get_base64_jpeg(quality=45)
                if b64:
                    payload["frame_b64"] = b64

            # Status bits
            payload["video_active"] = video_service.active if video_service else False
            payload["serial_connected"] = sensor_service.connected if sensor_service else False

            # Playback state (for control bar)
            if video_service:
                payload["playback"] = video_service.get_playback_state()

            # Live SLAM visualization frame
            if live_slam_service and live_slam_service.available:
                slam_b64 = live_slam_service.get_visualization_base64(quality=40)
                if slam_b64:
                    payload["slam_frame_b64"] = slam_b64
                payload["slam_stats"] = live_slam_service.get_stats()

            if payload:
                await ws.send_text(json.dumps(payload, default=str))

            await asyncio.sleep(0.25)  # ~4 updates/sec to frontend

    except WebSocketDisconnect:
        logger.info("WebSocket client disconnected")
    except Exception as e:
        logger.warning(f"WebSocket error: {e}")
