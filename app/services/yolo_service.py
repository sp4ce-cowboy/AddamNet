"""
yolo_service.py
─────────────────────────────────────────────────────────────────
Loads the trained YOLO model and runs inference on frames
provided by the video service.  Runs in a background thread
with configurable inference rate for Raspberry Pi performance.

Detections are passed to the defect_logger for event creation.
"""

import time
import threading
import logging
import base64
import cv2
import numpy as np
from typing import Optional, List, Dict, Any, Callable

logger = logging.getLogger("yolo_service")

# ── Try importing ultralytics ──
try:
    from ultralytics import YOLO
    HAS_YOLO = True
except ImportError:
    HAS_YOLO = False
    logger.warning("ultralytics not installed — YOLO features disabled")


class YOLOService:
    """
    Loads YOLO model, runs inference on latest frames, produces
    annotated frames and detection results.
    """

    def __init__(
        self,
        model_path: str,
        confidence_threshold: float = 0.35,
        imgsz: int = 320,
        class_names: Optional[Dict[int, str]] = None,
    ):
        self._model_path = model_path
        self._conf = confidence_threshold
        self._imgsz = imgsz
        self._class_names = class_names or {}
        self._model = None
        self._loaded = False

        # Latest inference results
        self._lock = threading.Lock()
        self._annotated_frame: Optional[np.ndarray] = None
        self._detections: List[Dict[str, Any]] = []
        self._inference_ms: float = 0
        self._frame_number: int = 0

        # Callback for new detections
        self.on_detections: Optional[Callable] = None

    @property
    def loaded(self) -> bool:
        return self._loaded

    def load_model(self):
        """Load the YOLO model. Call once at startup."""
        if not HAS_YOLO:
            logger.error("Cannot load model — ultralytics not installed")
            return False
        try:
            self._model = YOLO(self._model_path)
            self._loaded = True
            logger.info(f"YOLO model loaded: {self._model_path}")
            return True
        except Exception as e:
            logger.error(f"Failed to load YOLO model: {e}")
            return False

    def run_inference(self, frame: np.ndarray, frame_number: int) -> List[Dict[str, Any]]:
        """
        Run YOLO inference on a single BGR frame.
        Returns list of detection dicts and stores annotated frame.
        """
        if not self._loaded or self._model is None:
            return []

        t0 = time.time()

        try:
            results = self._model.predict(
                frame,
                conf=self._conf,
                imgsz=self._imgsz,
                verbose=False,
            )
        except Exception as e:
            logger.error(f"Inference error: {e}")
            return []

        elapsed_ms = (time.time() - t0) * 1000

        detections = []
        annotated = frame.copy()

        if results and len(results) > 0:
            result = results[0]
            boxes = result.boxes
            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cls_name = self._class_names.get(cls_id, f"class_{cls_id}")

                    detections.append({
                        "class_id": cls_id,
                        "class_name": cls_name,
                        "confidence": round(conf, 3),
                        "bbox": [int(x1), int(y1), int(x2), int(y2)],
                    })

                    # Draw on annotated frame
                    color = self._class_color(cls_id)
                    cv2.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                    label = f"{cls_name} {conf:.2f}"
                    (tw, th), _ = cv2.getTextSize(
                        label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
                    )
                    cv2.rectangle(
                        annotated,
                        (int(x1), int(y1) - th - 6),
                        (int(x1) + tw, int(y1)),
                        color,
                        -1,
                    )
                    cv2.putText(
                        annotated, label,
                        (int(x1), int(y1) - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
                        cv2.LINE_AA,
                    )

        # HUD
        cv2.putText(
            annotated,
            f"Inference: {elapsed_ms:.0f}ms | Detections: {len(detections)}",
            (10, 22),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA,
        )

        with self._lock:
            self._annotated_frame = annotated
            self._detections = detections
            self._inference_ms = elapsed_ms
            self._frame_number = frame_number

        # Notify callback
        if detections and self.on_detections:
            self.on_detections(detections, frame_number)

        return detections

    def get_annotated_jpeg(self, quality: int = 60) -> Optional[bytes]:
        """Return latest annotated frame as JPEG bytes."""
        with self._lock:
            frame = self._annotated_frame
        if frame is None:
            return None
        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return buf.tobytes()

    def get_annotated_base64(self, quality: int = 60) -> Optional[str]:
        with self._lock:
            frame = self._annotated_frame
        if frame is None:
            return None
        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return base64.b64encode(buf.tobytes()).decode("ascii")

    def get_latest_detections(self) -> List[Dict[str, Any]]:
        with self._lock:
            return list(self._detections)

    def get_inference_ms(self) -> float:
        with self._lock:
            return self._inference_ms

    @staticmethod
    def _class_color(cls_id: int):
        """Deterministic color per class."""
        colors = [
            (0, 0, 255),    # Deformation  - red
            (0, 165, 255),  # Obstacle     - orange
            (0, 255, 255),  # Rupture      - yellow
            (255, 0, 0),    # Disconnect   - blue
            (255, 0, 255),  # Misalignment - magenta
            (0, 255, 0),    # Deposition   - green
        ]
        return colors[cls_id % len(colors)]
