"""
slam_service.py
─────────────────────────────────────────────────────────────────
Wraps the existing visual SLAM + 3D reconstruction pipeline
(visual_slam.py, reconstruct_viewer.py, pipeline.py) for use
from the dashboard.

Current approach:
  - Run SLAM as a background process on a video file
  - Track progress / status
  - Expose generated output files (viewer HTML, PLY, trajectory)
  - Serve the generated HTML viewer via the dashboard

This is an integration point — the actual SLAM logic lives
unchanged in slam/visual_slam.py and slam/reconstruct_viewer.py.
"""

import os
import time
import threading
import logging
from datetime import datetime
from typing import Optional, List

from app.models.schemas import SLAMStatus

logger = logging.getLogger("slam_service")


class SLAMService:
    """
    Manages SLAM pipeline execution and output tracking.
    """

    def __init__(self, output_dir: str = "data/slam_output"):
        self._output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

        self._status = SLAMStatus(message="Idle — no SLAM run yet")
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        # Check for existing outputs from previous runs
        self._scan_existing_outputs()

    @property
    def available(self) -> bool:
        """Check if the SLAM modules can be imported."""
        try:
            import sys
            from app.config import project_root
            slam_dir = str(project_root() / "slam")
            if slam_dir not in sys.path:
                sys.path.insert(0, slam_dir)
            from visual_slam import run_slam
            from reconstruct_viewer import (
                read_ply, read_trajectory,
                statistical_outlier_removal, estimate_normals,
                reconstruct_surface, generate_html_viewer,
            )
            return True
        except ImportError as e:
            logger.warning(f"SLAM modules not available: {e}")
            return False

    def get_status(self) -> SLAMStatus:
        with self._lock:
            return self._status.model_copy()

    def start_run(self, input_video: str) -> bool:
        """
        Start a SLAM pipeline run on the given video file.
        Runs in a background thread so dashboard stays responsive.
        """
        if self._status.running:
            logger.warning("SLAM already running")
            return False

        if not os.path.isfile(input_video):
            logger.error(f"Input video not found: {input_video}")
            return False

        with self._lock:
            self._status.running = True
            self._status.progress_pct = 0.0
            self._status.message = "Starting SLAM pipeline..."
            self._status.output_files = []

        self._thread = threading.Thread(
            target=self._run_pipeline,
            args=(input_video,),
            daemon=True,
        )
        self._thread.start()
        return True

    def _run_pipeline(self, input_video: str):
        """Execute the full SLAM + reconstruction pipeline."""
        import sys
        from app.config import project_root

        slam_dir = str(project_root() / "slam")
        if slam_dir not in sys.path:
            sys.path.insert(0, slam_dir)

        try:
            video_out = os.path.join(self._output_dir, "slam_output.mp4")
            viewer_out = os.path.join(self._output_dir, "slam_viewer.html")

            # ── Step 1: Visual SLAM ──
            with self._lock:
                self._status.message = "Running Visual SLAM..."
                self._status.progress_pct = 10.0

            from visual_slam import run_slam
            ply_path, traj_path = run_slam(input_video, video_out)

            with self._lock:
                self._status.progress_pct = 60.0
                self._status.message = "SLAM complete. Reconstructing 3D..."

            # ── Step 2: 3D Reconstruction ──
            from reconstruct_viewer import (
                read_ply, read_trajectory,
                statistical_outlier_removal, estimate_normals,
                reconstruct_surface, generate_html_viewer,
            )
            import numpy as np

            pts = read_ply(ply_path)
            trajectory = read_trajectory(traj_path)

            pts_clean, _ = statistical_outlier_removal(pts, k=20, std_ratio=2.0)
            normals = estimate_normals(pts_clean, k=15)

            with self._lock:
                self._status.progress_pct = 80.0
                self._status.message = "Generating 3D viewer..."

            faces = np.empty((0, 3), dtype=int)
            if len(pts_clean) >= 4:
                try:
                    faces = reconstruct_surface(pts_clean, normals, alpha=None)
                except Exception as e:
                    logger.warning(f"Surface reconstruction failed: {e}")

            html = generate_html_viewer(
                pts_clean, normals, faces, trajectory,
                title="Pipeline SLAM 3D Reconstruction",
            )
            with open(viewer_out, "w") as f:
                f.write(html)

            # ── Done ──
            outputs = [p for p in [video_out, ply_path, traj_path, viewer_out]
                       if os.path.exists(p)]

            with self._lock:
                self._status.running = False
                self._status.progress_pct = 100.0
                self._status.message = "SLAM run complete"
                self._status.last_run_time = datetime.utcnow().isoformat() + "Z"
                self._status.viewer_html_path = viewer_out
                self._status.output_files = outputs

            logger.info(f"SLAM pipeline complete — {len(outputs)} output files")

        except Exception as e:
            logger.error(f"SLAM pipeline failed: {e}", exc_info=True)
            with self._lock:
                self._status.running = False
                self._status.progress_pct = 0.0
                self._status.message = f"SLAM failed: {e}"

    def _scan_existing_outputs(self):
        """Check for output files from a previous run."""
        viewer = os.path.join(self._output_dir, "slam_viewer.html")
        if os.path.isfile(viewer):
            self._status.viewer_html_path = viewer
            self._status.message = "Previous SLAM output found"
            # Collect all outputs
            for fname in os.listdir(self._output_dir):
                fpath = os.path.join(self._output_dir, fname)
                if os.path.isfile(fpath):
                    self._status.output_files.append(fpath)

    def get_viewer_html_path(self) -> Optional[str]:
        """Return path to the generated HTML viewer, or None."""
        with self._lock:
            return self._status.viewer_html_path
