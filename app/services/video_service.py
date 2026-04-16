"""
video_service.py
─────────────────────────────────────────────────────────────────
Abstracts the video source (USB webcam or pre-recorded file).
Captures frames in a background thread and provides the latest
frame to the YOLO inference pipeline.

Supports play/pause/seek for pre-recorded video files.

Designed for Raspberry Pi 5 constraints:
  - configurable target FPS (frame skipping)
  - configurable resolution downscale
  - thread-safe frame buffer
"""

import cv2
import time
import threading
import logging
import base64
import numpy as np
from typing import Optional, Tuple, Dict, Any

logger = logging.getLogger("video_service")


class VideoService:
    def __init__(
        self,
        source: str = "webcam",
        webcam_device=0,
        video_file: str = "",
        target_fps: float = 5.0,
        frame_width: int = 640,
        frame_height: int = 480,
    ):
        self._source_type = source
        self._webcam_device = webcam_device
        self._video_file = video_file
        self._target_fps = max(1.0, target_fps)
        self._frame_width = frame_width
        self._frame_height = frame_height

        self._cap: Optional[cv2.VideoCapture] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._active = False

        self._lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_number: int = 0
        self._source_label: str = "none"

        # Playback control
        self._paused = False
        self._seek_target: Optional[int] = None
        self._total_frames: int = 0
        self._video_fps: float = 30.0
        self._current_pos_frame: int = 0
        self._switch_to: Optional[str] = None  # "webcam" or "file"

    @property
    def active(self) -> bool:
        return self._active

    @property
    def source_label(self) -> str:
        return self._source_label

    @property
    def frame_number(self) -> int:
        return self._frame_number

    @property
    def paused(self) -> bool:
        return self._paused

    @property
    def is_file_source(self) -> bool:
        return self._source_type == "file"

    @property
    def video_file_path(self) -> str:
        return self._video_file

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._cap and self._cap.isOpened():
            self._cap.release()
        self._active = False

    def get_frame(self) -> Tuple[Optional[np.ndarray], int]:
        with self._lock:
            if self._latest_frame is not None:
                return self._latest_frame.copy(), self._frame_number
            return None, 0

    def get_jpeg(self, quality: int = 60) -> Optional[bytes]:
        frame, _ = self.get_frame()
        if frame is None:
            return None
        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return buf.tobytes()

    def get_base64_jpeg(self, quality: int = 60) -> Optional[str]:
        jpg = self.get_jpeg(quality)
        if jpg is None:
            return None
        return base64.b64encode(jpg).decode("ascii")

    # ── Playback controls ──

    def pause(self):
        self._paused = True

    def resume(self):
        self._paused = False

    def toggle_pause(self) -> bool:
        self._paused = not self._paused
        return self._paused

    def seek(self, frame_num: int):
        frame_num = max(0, min(frame_num, max(self._total_frames - 1, 0)))
        self._seek_target = frame_num
        self._paused = False

    def seek_fraction(self, fraction: float):
        fraction = max(0.0, min(1.0, fraction))
        target = int(fraction * max(self._total_frames - 1, 0))
        self.seek(target)

    def switch_source(self, source_type: str) -> str:
        """Request a hot-swap to 'webcam' or 'file'. Returns the new source type."""
        if source_type not in ("webcam", "file"):
            return self._source_type
        if source_type == self._source_type:
            return self._source_type
        self._switch_to = source_type
        return source_type

    def set_video_file(self, path: str):
        """Change the video file path and switch to file mode."""
        self._video_file = path
        self._switch_to = "file"

    def get_playback_state(self) -> Dict[str, Any]:
        duration_s = self._total_frames / self._video_fps if self._video_fps > 0 else 0
        current_s = self._current_pos_frame / self._video_fps if self._video_fps > 0 else 0
        progress = self._current_pos_frame / max(self._total_frames, 1)
        return {
            "paused": self._paused,
            "is_file": self.is_file_source,
            "source_type": self._source_type,
            "current_frame": self._current_pos_frame,
            "total_frames": self._total_frames,
            "current_s": round(current_s, 1),
            "duration_s": round(duration_s, 1),
            "progress": round(progress, 4),
        }

    # ── Internal ──

    def _open_source(self) -> bool:
        if self._source_type == "file" and self._video_file:
            self._cap = cv2.VideoCapture(self._video_file)
            self._source_label = f"file:{self._video_file}"
        else:
            self._cap = cv2.VideoCapture(self._webcam_device)
            if self._cap.isOpened():
                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._frame_width)
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._frame_height)
            self._source_label = f"webcam:{self._webcam_device}"

        if not self._cap.isOpened():
            logger.warning(f"Cannot open video source: {self._source_label}")
            return False

        self._active = True
        self._total_frames = int(self._cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 0
        self._video_fps = self._cap.get(cv2.CAP_PROP_FPS) or 30.0
        logger.info(f"Video opened: {self._source_label} "
                     f"({self._total_frames} frames @ {self._video_fps:.1f} fps)")
        return True

    def _capture_loop(self):
        if not self._open_source():
            if self._source_type == "webcam" and self._video_file:
                logger.info("Webcam unavailable, falling back to video file")
                self._source_type = "file"
                if not self._open_source():
                    logger.error("No video source available")
                    return
            else:
                return

        interval = 1.0 / self._target_fps

        while self._running:
            t0 = time.time()

            # Handle source switch
            new_src = self._switch_to
            if new_src is not None:
                self._switch_to = None
                self._source_type = new_src
                if self._cap and self._cap.isOpened():
                    self._cap.release()
                self._current_pos_frame = 0
                self._frame_number = 0
                self._paused = False
                if not self._open_source():
                    logger.warning(f"Switch to {new_src} failed, staying inactive")
                    time.sleep(0.5)
                    continue
                logger.info(f"Switched to {new_src}")

            # Handle seek
            seek = self._seek_target
            if seek is not None:
                self._seek_target = None
                self._cap.set(cv2.CAP_PROP_POS_FRAMES, seek)
                self._current_pos_frame = seek

            # Handle pause
            if self._paused:
                time.sleep(0.05)
                continue

            ret, frame = self._cap.read()

            if not ret:
                if self._source_type == "file":
                    self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    self._current_pos_frame = 0
                    continue
                else:
                    time.sleep(0.5)
                    continue

            self._current_pos_frame = int(self._cap.get(cv2.CAP_PROP_POS_FRAMES))

            h, w = frame.shape[:2]
            if w != self._frame_width or h != self._frame_height:
                frame = cv2.resize(frame, (self._frame_width, self._frame_height),
                                   interpolation=cv2.INTER_AREA)

            with self._lock:
                self._latest_frame = frame
                self._frame_number += 1

            elapsed = time.time() - t0
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        if self._cap:
            self._cap.release()
        self._active = False
