"""
live_slam_service.py
─────────────────────────────────────────────────────────────────
Wraps the existing VisualSLAM + SLAMVisualizer classes for
real-time frame-by-frame processing.  Feeds frames from the
video service, produces a composite visualization frame
(tracked features + trajectory + 3D map), and exposes it for
streaming to the dashboard.

Performance notes for Raspberry Pi 5:
  - KLT tracking + essential matrix ≈ 10-30 ms per frame
  - Much lighter than YOLO, so both can coexist
  - Runs at a configurable fraction of the video FPS
    (e.g. every 3rd frame) to save CPU
"""

import sys
import cv2
import threading
import logging
import base64
import numpy as np
from pathlib import Path
from typing import Optional

logger = logging.getLogger("live_slam_service")


class LiveSLAMService:
    """
    Real-time SLAM visualization that processes frames incrementally
    and produces a composite output image each time.
    """

    def __init__(self, frame_width: int = 640, frame_height: int = 480, skip_frames: int = 2):
        self._frame_width = frame_width
        self._frame_height = frame_height
        self._skip_frames = skip_frames  # process every Nth frame (1 = every frame)

        self._slam = None
        self._viz = None
        self._initialized = False
        self._frame_counter = 0

        self._lock = threading.Lock()
        self._latest_viz_frame: Optional[np.ndarray] = None
        self._stats = {"map_pts": 0, "trajectory_len": 0, "keyframes": 0}

        self._init_slam()

    def _init_slam(self):
        """Import and instantiate the SLAM classes from slam/visual_slam.py."""
        try:
            from app.config import project_root
            slam_dir = str(project_root() / "slam")
            if slam_dir not in sys.path:
                sys.path.insert(0, slam_dir)

            from visual_slam import VisualSLAM, SLAMVisualizer, SLAMConfig, make_camera_matrix

            w, h = self._frame_width, self._frame_height
            K = make_camera_matrix(w, h, fov_deg=60)

            cfg = SLAMConfig()
            self._slam = VisualSLAM(K, cfg)
            self._viz = SLAMVisualizer(w, h, cfg)
            self._initialized = True
            logger.info(f"Live SLAM initialized ({w}x{h})")
        except ImportError as e:
            logger.warning(f"Could not initialize live SLAM: {e}")
        except Exception as e:
            logger.error(f"Live SLAM init failed: {e}", exc_info=True)

    @property
    def available(self) -> bool:
        return self._initialized

    @property
    def stats(self) -> dict:
        with self._lock:
            return dict(self._stats)

    def process_frame(self, frame: np.ndarray) -> bool:
        """
        Feed a BGR frame into the SLAM pipeline.
        Returns True if the frame was actually processed (not skipped).
        """
        if not self._initialized:
            return False

        self._frame_counter += 1

        # Skip frames to reduce CPU load
        if self._skip_frames > 1 and (self._frame_counter % self._skip_frames) != 0:
            return False

        try:
            # Resize to expected dimensions if needed
            h, w = frame.shape[:2]
            if w != self._frame_width or h != self._frame_height:
                frame = cv2.resize(frame, (self._frame_width, self._frame_height),
                                   interpolation=cv2.INTER_AREA)

            # Run SLAM on this frame
            info = self._slam.process_frame(frame)

            # Generate composite visualization
            viz_frame = self._viz.draw(frame, self._slam, info)

            with self._lock:
                self._latest_viz_frame = viz_frame
                self._stats = {
                    "map_pts": info.get("num_map_pts", 0),
                    "trajectory_len": len(self._slam.trajectory),
                    "keyframes": len(self._slam.keyframes) if hasattr(self._slam, 'keyframes') else 0,
                    "matches": info.get("num_matches", 0),
                    "inliers": info.get("num_inliers", 0),
                }

            return True

        except Exception as e:
            logger.error(f"Live SLAM frame error: {e}")
            return False

    def get_viz_jpeg(self, quality: int = 50) -> Optional[bytes]:
        """Return latest SLAM visualization as JPEG bytes."""
        with self._lock:
            frame = self._latest_viz_frame
        if frame is None:
            return None
        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return buf.tobytes()

    def get_viz_base64(self, quality: int = 45) -> Optional[str]:
        """Return latest SLAM visualization as base64 JPEG."""
        jpg = self.get_viz_jpeg(quality)
        if jpg is None:
            return None
        return base64.b64encode(jpg).decode("ascii")

    def reset(self):
        """Reset SLAM state (e.g. when switching video sources)."""
        logger.info("Resetting live SLAM state")
        self._frame_counter = 0
        with self._lock:
            self._latest_viz_frame = None
            self._stats = {"map_pts": 0, "trajectory_len": 0, "keyframes": 0}
        self._init_slam()
