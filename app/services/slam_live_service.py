"""
slam_live_service.py
─────────────────────────────────────────────────────────────────
Wraps VisualSLAM + SLAMVisualizer for real-time per-frame
processing.  Produces a composite SLAM visualization frame
(tracked features + trajectory + 3D map panel) that can be
streamed to the dashboard alongside the YOLO feed.

Lazy-initializes on the first frame (needs dimensions to build
the camera matrix and visualizer).

Performance note for RPi 5:
  SLAM processing per frame is mostly OpenCV operations (KLT
  optical flow, essential matrix, triangulation) — lighter than
  YOLO but still significant.  Use `process_every_n` to skip
  frames and keep CPU usage reasonable.
"""

import sys
import threading
import logging
import time
import base64
import cv2
import numpy as np
from pathlib import Path
from typing import Optional, Dict, Any

logger = logging.getLogger("slam_live")


class SLAMLiveService:
    """
    Real-time SLAM visualization from individual frames.
    """

    def __init__(self, slam_module_dir: str, process_every_n: int = 2):
        self._slam_dir = slam_module_dir
        self._every_n = max(1, process_every_n)

        self._slam = None          # VisualSLAM instance
        self._viz = None           # SLAMVisualizer instance
        self._initialized = False
        self._available = False
        self._frame_counter = 0

        self._lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_info: Dict[str, Any] = {}

        # Check if SLAM modules can be imported
        self._ensure_importable()

    def _ensure_importable(self):
        """Put the slam/ directory on sys.path so we can import."""
        if self._slam_dir not in sys.path:
            sys.path.insert(0, self._slam_dir)
        try:
            from visual_slam import VisualSLAM, SLAMVisualizer, make_camera_matrix, SLAMConfig
            self._available = True
            logger.info("SLAM live modules available")
        except ImportError as e:
            logger.warning(f"SLAM live modules not importable: {e}")
            self._available = False

    @property
    def available(self) -> bool:
        return self._available

    @property
    def initialized(self) -> bool:
        return self._initialized

    def process_frame(self, frame_bgr: np.ndarray) -> Optional[np.ndarray]:
        """
        Process a single BGR frame through SLAM.
        Returns the composite visualization frame, or None if skipped.
        """
        if not self._available:
            return None

        self._frame_counter += 1

        # Skip frames to save CPU
        if self._frame_counter % self._every_n != 0:
            # Return the last visualization frame (stale but avoids flicker)
            with self._lock:
                return self._latest_frame.copy() if self._latest_frame is not None else None

        # Lazy init on first real frame
        if not self._initialized:
            self._init_slam(frame_bgr.shape[1], frame_bgr.shape[0])

        if not self._initialized:
            return None

        try:
            info = self._slam.process_frame(frame_bgr)
            vis = self._viz.draw(frame_bgr, self._slam, info)

            with self._lock:
                self._latest_frame = vis
                self._latest_info = info

            return vis

        except Exception as e:
            logger.error(f"SLAM frame processing error: {e}")
            return None

    def _init_slam(self, w: int, h: int):
        """Initialize SLAM engine and visualizer with frame dimensions."""
        try:
            from visual_slam import VisualSLAM, SLAMVisualizer, make_camera_matrix, SLAMConfig

            cfg = SLAMConfig()
            K = make_camera_matrix(w, h, fov_deg=60)
            self._slam = VisualSLAM(K, cfg)
            self._viz = SLAMVisualizer(w, h, cfg)
            self._initialized = True
            logger.info(f"SLAM live initialized: {w}x{h}")
        except Exception as e:
            logger.error(f"SLAM live init failed: {e}")
            self._initialized = False

    def get_visualization_jpeg(self, quality: int = 50) -> Optional[bytes]:
        """Return latest SLAM visualization as JPEG bytes."""
        with self._lock:
            frame = self._latest_frame
        if frame is None:
            return None
        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return buf.tobytes()

    def get_visualization_base64(self, quality: int = 45) -> Optional[str]:
        """Return latest SLAM visualization as base64 JPEG."""
        jpg = self.get_visualization_jpeg(quality)
        if jpg is None:
            return None
        return base64.b64encode(jpg).decode("ascii")

    def get_stats(self) -> Dict[str, Any]:
        """Return current SLAM statistics."""
        if not self._initialized or self._slam is None:
            return {"initialized": False}
        with self._lock:
            info = dict(self._latest_info)
        return {
            "initialized": True,
            "trajectory_length": len(self._slam.trajectory),
            "map_points": len(self._slam.map_points),
            "matches": info.get("num_matches", 0),
            "inliers": info.get("num_inliers", 0),
        }

    def has_data(self) -> bool:
        """Return True if there is enough SLAM data to export."""
        if not self._initialized or self._slam is None:
            return False
        return len(self._slam.map_points) > 10 and len(self._slam.trajectory) > 2

    def export_results(self, output_dir: str) -> Dict[str, Any]:
        """
        Export the current live SLAM state to files:
          - PLY point cloud
          - CSV trajectory
          - HTML 3D viewer (via reconstruct_viewer)
        Returns dict with output file paths and status.
        """
        import os
        os.makedirs(output_dir, exist_ok=True)

        if not self.has_data():
            return {"success": False, "error": "Not enough SLAM data yet"}

        try:
            from reconstruct_viewer import (
                statistical_outlier_removal, estimate_normals,
                reconstruct_surface, generate_html_viewer,
            )

            # Snapshot current state
            pts = np.array(list(self._slam.map_points))
            traj = np.array(self._slam.trajectory)

            # Save PLY
            ply_path = os.path.join(output_dir, "live_slam_map.ply")
            with open(ply_path, 'w') as f:
                f.write(f"ply\nformat ascii 1.0\nelement vertex {len(pts)}\n")
                f.write("property float x\nproperty float y\nproperty float z\nend_header\n")
                for p in pts:
                    f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

            # Save trajectory CSV
            traj_path = os.path.join(output_dir, "live_slam_trajectory.csv")
            np.savetxt(traj_path, traj, delimiter=',', header='x,y,z', comments='')

            # Process and generate viewer
            pts_clean, _ = statistical_outlier_removal(pts, k=min(20, len(pts) - 1), std_ratio=2.0)
            normals = estimate_normals(pts_clean, k=min(15, len(pts_clean) - 1))

            faces = np.empty((0, 3), dtype=int)
            if len(pts_clean) >= 4:
                try:
                    faces = reconstruct_surface(pts_clean, normals, alpha=None)
                except Exception:
                    pass

            html = generate_html_viewer(
                pts_clean, normals, faces, traj,
                title="Live SLAM 3D Reconstruction",
            )
            viewer_path = os.path.join(output_dir, "slam_viewer.html")
            with open(viewer_path, 'w') as f:
                f.write(html)

            outputs = [p for p in [ply_path, traj_path, viewer_path] if os.path.exists(p)]
            logger.info(f"Live SLAM exported: {len(pts)} pts, {len(traj)} poses → {output_dir}")

            return {
                "success": True,
                "map_points": len(pts),
                "trajectory_poses": len(traj),
                "output_files": outputs,
                "viewer_html": viewer_path,
            }

        except Exception as e:
            logger.error(f"Live SLAM export failed: {e}", exc_info=True)
            return {"success": False, "error": str(e)}
