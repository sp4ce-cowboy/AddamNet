> [!NOTE]
> This is the project's original README, from the project itself (Aug 2025 – Apr 2026).
> Claude Opus 5.5 moved it here in Nov 2026, as part of my
> [agentic-repo-analysis](https://github.com/sp4ce-cowboy/agentic-workflows) skill, when the
> consolidated [README](README.md) took its place. It is otherwise unchanged.

# Pipeline Inspection Dashboard

Integrated dashboard for autonomous pipeline inspection — combining embedded
sensing, AI defect detection, and visual SLAM into a single Raspberry Pi 5
application.

---

## Project Structure

```
pipeline_inspector/
├── run.py                          ← Entry point: starts the dashboard server
├── config.yaml                     ← Central configuration (edit this)
├── requirements.txt                ← Python dependencies
├── README.md
│
├── app/
│   ├── __init__.py
│   ├── main.py                     ← FastAPI app: wires services, lifespan
│   ├── config.py                   ← YAML config loader
│   │
│   ├── models/
│   │   ├── __init__.py
│   │   └── schemas.py              ← Pydantic models (SensorSnapshot, DefectEvent, etc.)
│   │
│   ├── services/
│   │   ├── __init__.py
│   │   ├── sensor_service.py       ← Arduino serial reader (background thread)
│   │   ├── video_service.py        ← Webcam / video file capture (background thread)
│   │   ├── yolo_service.py         ← YOLO inference + annotation
│   │   ├── defect_logger.py        ← SQLite defect event storage with cooldown
│   │   └── slam_service.py         ← SLAM pipeline wrapper
│   │
│   ├── api/
│   │   ├── __init__.py
│   │   └── routes.py               ← REST + WebSocket endpoints
│   │
│   └── static/
│       ├── index.html              ← Dashboard UI (single-page, 5 tabs)
│       ├── style.css               ← Dark industrial theme
│       └── app.js                  ← WebSocket client + DOM updates
│
├── slam/
│   ├── __init__.py
│   ├── visual_slam.py              ← YOUR existing SLAM code (unchanged)
│   ├── reconstruct_viewer.py       ← YOUR existing 3D viewer generator (unchanged)
│   └── pipeline.py                 ← YOUR existing pipeline runner (unchanged)
│
├── weights/
│   └── best.pt                     ← YOUR trained YOLO model weights
│
├── arduino/
│   ├── sensor_suite.ino            ← YOUR Arduino sketch (reference copy)
│   └── sample_sensor_readings.txt  ← Sample JSON output for reference
│
└── data/
    ├── defects.db                  ← SQLite database (auto-created at runtime)
    └── slam_output/                ← SLAM output directory (auto-created)
```

---

## Quick Start on Raspberry Pi 5

### 1. Install dependencies

```bash
# Option A: System-wide (simpler)
pip install -r requirements.txt --break-system-packages

# Option B: Virtual environment (cleaner)
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### 2. Edit config.yaml

Open `config.yaml` and adjust for your setup:

```yaml
video:
  source: "webcam"          # "webcam" for live camera, "file" for pre-recorded
  webcam_device: 0           # usually 0 for /dev/video0
  video_file: "/path/to/your/video.mp4"  # used when source is "file"
  target_fps: 5              # lower = less CPU usage

serial:
  port: "/dev/ttyUSB0"      # or "auto" to detect, or "mock" for fake data
```

### 3. Run

```bash
cd pipeline_inspector
python3 run.py
```

### 4. Open the dashboard

Navigate to `http://<raspberry-pi-ip>:8000` in any browser on the same network.

---

## How Each Subsystem Connects

### Architecture Diagram

```
┌───────────────────────────────────────────────────────────┐
│                     Raspberry Pi 5                         │
│                                                           │
│  ┌──────────┐    serial     ┌───────────────┐            │
│  │ Arduino  │──────────────▶│ sensor_service │            │
│  │  Mega    │   JSON@115200 │  (bg thread)   │            │
│  └──────────┘               └───────┬───────┘            │
│                                     │ latest snapshot     │
│  ┌──────────┐    OpenCV     ┌───────▼───────┐            │
│  │  USB     │──────────────▶│ video_service  │            │
│  │  Webcam  │   frames      │  (bg thread)   │            │
│  └──────────┘               └───────┬───────┘            │
│                                     │ frames              │
│                             ┌───────▼───────┐            │
│  ┌──────────┐               │ yolo_service   │            │
│  │ best.pt  │──────────────▶│ (inference     │            │
│  │ (YOLO)   │   model       │  bg thread)    │            │
│  └──────────┘               └──┬────┬───────┘            │
│                     detections │    │ annotated frames    │
│                    ┌───────────▼┐   │                    │
│                    │defect_logger│   │                    │
│                    │  (SQLite)   │   │                    │
│                    └─────────────┘   │                    │
│                                     │                    │
│                    ┌────────────────▼────────────┐       │
│                    │       FastAPI Server         │       │
│                    │  REST API + WebSocket        │       │
│                    │  /api/* + /ws/live           │       │
│                    └────────────┬────────────────┘       │
│                                │                         │
└────────────────────────────────┼─────────────────────────┘
                                 │  HTTP / WS
                    ┌────────────▼────────────┐
                    │     Browser Dashboard    │
                    │  Live Feed │ Sensors     │
                    │  Defects   │ SLAM        │
                    │  System Status           │
                    └─────────────────────────┘
```

### Data Flow

1. **sensor_service** runs a background thread that reads JSON lines from the
   Arduino Mega's serial port. Each JSON line is parsed into a `SensorSnapshot`
   and stored in memory as `sensor_service.latest`. The WebSocket pushes this
   to the frontend at ~4 Hz.

2. **video_service** runs a background thread that captures frames from either
   the USB webcam (`cv2.VideoCapture(0)`) or a video file. Frames are
   downscaled and rate-limited to the configured `target_fps`. The latest
   frame is stored thread-safely.

3. **yolo_service** loads `best.pt` at startup. A separate inference loop in
   `main.py` pulls the latest frame from `video_service`, runs YOLO detection,
   draws bounding boxes, and stores the annotated frame. The WebSocket sends
   the annotated frame as base64 JPEG.

4. **defect_logger** is called whenever YOLO produces detections. It applies
   a per-class cooldown (default 5 seconds) to avoid spamming duplicate events,
   then inserts a `DefectEvent` into the SQLite database along with the current
   sensor snapshot.

5. **slam_service** wraps your existing `visual_slam.py` and
   `reconstruct_viewer.py`. It can run the full SLAM pipeline on a video file
   in a background thread, tracking progress. The generated HTML viewer is
   served at `/slam-viewer`.

---

## How to Plug In the Real Arduino JSON Stream

Your Arduino Mega sends JSON at 115200 baud over USB serial. The format from
your `sample_sensor_readings.txt` is already fully supported.

**Steps:**

1. Plug the Arduino Mega into the Raspberry Pi via USB.
2. Find the port:
   ```bash
   ls /dev/ttyUSB* /dev/ttyACM*
   ```
   It will typically be `/dev/ttyUSB0` or `/dev/ttyACM0`.

3. Set the port in `config.yaml`:
   ```yaml
   serial:
     port: "/dev/ttyUSB0"   # or "auto" to auto-detect
   ```

4. If you get permission errors:
   ```bash
   sudo usermod -aG dialout $USER
   # then log out and back in
   ```

5. The dashboard will immediately start showing live sensor values.

**Mock mode:** If the Arduino is not connected and `mock_fallback: true` is set
(the default), the system generates realistic fake sensor data so the dashboard
still works for development and demonstration.

---

## How to Swap Between Live Webcam and Pre-recorded Video

Edit `config.yaml`:

```yaml
video:
  # For live USB webcam:
  source: "webcam"
  webcam_device: 0

  # For pre-recorded video:
  source: "file"
  video_file: "/home/pi/videos/sewage.mp4"
```

The same YOLO inference pipeline runs on both sources. When using a video file,
it loops automatically when it reaches the end.

**Fallback:** If `source: "webcam"` but the webcam is unavailable, and a
`video_file` path is configured, the system automatically falls back to the
video file.

---

## How Defect Events Are Recorded

When YOLO detects one or more objects in a frame:

1. Each detection has a `class_name` and `confidence` score.

2. The **cooldown mechanism** checks: "Has this class been logged in the last
   N seconds?" (configurable via `defects.cooldown_seconds`, default 5s).
   If yes, the detection is skipped. This prevents the log from being flooded
   with duplicate entries when the camera sees the same defect across many
   consecutive frames.

3. If the cooldown has elapsed, a `DefectEvent` is created containing:
   - Timestamp
   - Defect class name (Deformation, Obstacle, Rupture, Disconnect,
     Misalignment, or Deposition)
   - Confidence score
   - Frame number
   - Video source identifier
   - **Current sensor snapshot** (temperature, humidity, distance readings,
     speed, odometer, air quality) — this is the key integration point between
     the AI and embedded subsystems.

4. The event is inserted into the SQLite database at `data/defects.db`.

5. The Defects tab on the dashboard shows all events in a sortable table,
   with per-class statistics and color-coded rows.

---

## How to Later Attach the Real SLAM Pipeline

The SLAM integration is designed as a practical integration point that can
be upgraded incrementally:

### Current capability (already working):
- The SLAM tab shows pipeline status and output files.
- You can trigger a SLAM run on any video file from the dashboard.
- The run executes your `visual_slam.py` → `reconstruct_viewer.py` pipeline
  in a background thread with progress tracking.
- The generated 3D HTML viewer is served directly at `/slam-viewer`.

### To run SLAM:
1. Go to the SLAM tab in the dashboard.
2. Enter the full path to a video file on the Pi.
3. Click "Run SLAM".
4. Watch the progress bar. When complete, click "Open 3D Viewer".

Alternatively via API:
```bash
curl -X POST "http://localhost:8000/api/slam/run?video_path=/home/pi/videos/pipe.mp4"
```

### Future upgrade path to semi-live SLAM:
The `slam_service.py` module is structured so you can add:
- A method that processes frames incrementally (from `video_service`)
- Status updates pushed via the existing WebSocket
- Periodic regeneration of the HTML viewer

The SLAM modules in `slam/` are your original files, completely unchanged.
Any improvements you make to them will automatically be used by the dashboard.

---

## Performance Notes for Raspberry Pi 5

### Expected bottlenecks:
1. **YOLO inference** — The biggest CPU consumer. At `imgsz=320` and
   `target_fps=5`, expect ~200-400ms per frame on the Pi 5 CPU. This is
   manageable at 2-5 FPS.

2. **Video capture** — OpenCV's VideoCapture is lightweight. The
   `target_fps` setting skips frames so the camera doesn't pile up a buffer.

3. **WebSocket streaming** — Base64 JPEG encoding adds ~5-10ms per frame.
   The JPEG quality is set to 45 for WebSocket to minimize bandwidth.

4. **SLAM processing** — This is offline (runs on pre-recorded video), so it
   doesn't compete with live inference. It will take several minutes for a
   typical video.

### Tuning for performance:
- Lower `target_fps` in config (3 FPS is still very usable)
- Lower `yolo.imgsz` to 256 (faster inference, slightly less accuracy)
- Lower `video.frame_width` / `frame_height` to 480×360
- The sensor service is very lightweight (~1 Hz serial reads)

### Memory:
On a 4GB Pi 5, the YOLO model (~6MB) plus OpenCV frames plus the web server
should comfortably fit within 1-2GB. The remaining memory is available for
SLAM processing when triggered.

---

## Where Each of Your Files Went

| Your original file           | Location in project                    | Changes made                          |
|------------------------------|----------------------------------------|---------------------------------------|
| `serial_reader.py`           | Logic adapted into `app/services/sensor_service.py` | Wrapped as threaded service, added mock mode, JSON parsing to Pydantic model |
| `sensor_suite.ino`           | `arduino/sensor_suite.ino`             | Unchanged (reference copy)            |
| `sample_sensor_readings.txt` | `arduino/sample_sensor_readings.txt`   | Unchanged (reference copy)            |
| `best.pt`                    | `weights/best.pt`                      | Unchanged (YOLO weights)              |
| `visual_slam.py`             | `slam/visual_slam.py`                  | Unchanged                             |
| `reconstruct_viewer.py`      | `slam/reconstruct_viewer.py`           | Unchanged                             |
| `pipeline.py`                | `slam/pipeline.py`                     | Unchanged (can still be run standalone) |

---

## Assumptions

1. **Raspberry Pi 5 (4GB)** running Raspberry Pi OS (Bookworm, 64-bit).
2. **Python 3.11+** available system-wide.
3. **USB webcam** at `/dev/video0` (Logitech UVC). If absent, mock/file mode.
4. **Arduino Mega** at `/dev/ttyUSB0` sending JSON at 115200 baud. If absent,
   mock sensor data is generated.
5. **No GPU** — YOLO runs on CPU. The `imgsz=320` setting keeps inference
   manageable on the Pi's ARM Cortex-A76 cores.
6. **Network access** — The dashboard is served on port 8000 and is accessible
   from any device on the same LAN.
7. **SLAM is offline** — Runs on pre-recorded video, not live feed. This is
   by design for the Pi's resource constraints.

---

## API Reference

| Method | Endpoint               | Description                              |
|--------|------------------------|------------------------------------------|
| GET    | `/`                    | Dashboard HTML                           |
| GET    | `/api/sensors`         | Latest sensor snapshot (JSON)            |
| GET    | `/api/defects?limit=N` | Recent defect events (JSON array)        |
| GET    | `/api/status`          | System status summary                    |
| GET    | `/api/slam`            | SLAM pipeline status                     |
| POST   | `/api/slam/run?video_path=...` | Trigger SLAM run                 |
| GET    | `/api/feed/frame`      | Latest annotated JPEG frame              |
| GET    | `/api/detections`      | Current frame detections                 |
| GET    | `/slam-viewer`         | Serve generated SLAM 3D HTML viewer      |
| WS     | `/ws/live`             | Real-time sensor + frame + detection stream |
