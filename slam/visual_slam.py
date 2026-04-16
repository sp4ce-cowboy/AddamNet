#!/usr/bin/env python3
"""
Monocular Visual SLAM Pipeline
===============================
Processes an input .mp4 video (e.g. robot POV in a pipeline network) and produces
an output video with:
  - Tracked ORB features overlaid on original frames
  - Estimated camera trajectory (top-down 2D plot)
  - Sparse 3D map point cloud (projected view)

Pipeline stages:
  1. ORB feature detection
  2. Feature matching (BFMatcher + ratio test) or KLT optical flow tracking
  3. Essential matrix estimation (5-point + RANSAC)
  4. Pose recovery (R, t)
  5. Triangulation of matched inlier points → 3D map points
  6. Keyframe management & local map maintenance
  7. Composite output video rendering

Usage:
    python3 visual_slam.py input.mp4 [output.mp4]
"""

import sys
import cv2
import numpy as np
from collections import deque


# ─────────────────────────── Configuration ───────────────────────────

class SLAMConfig:
    # Feature detection
    MAX_FEATURES = 2000
    ORB_SCALE_FACTOR = 1.2
    ORB_LEVELS = 8

    # Matching
    MATCH_RATIO_THRESH = 0.75      # Lowe's ratio test
    MIN_MATCHES = 30               # minimum matches to estimate pose

    # KLT Optical Flow (alternative tracker)
    USE_KLT = True                 # KLT is faster and better for video
    KLT_WIN_SIZE = (21, 21)
    KLT_MAX_LEVEL = 3
    KLT_CRITERIA = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
    FAST_THRESHOLD = 20
    MIN_TRACKED_POINTS = 200
    REDETECT_THRESHOLD = 400       # re-detect when tracked points drop below

    # RANSAC
    RANSAC_PROB = 0.999
    RANSAC_THRESH = 1.0

    # Keyframes
    KEYFRAME_MIN_PARALLAX = 1.5    # degrees of median parallax
    KEYFRAME_MIN_INLIERS = 50

    # Triangulation filter
    TRIANG_MAX_REPROJ_ERR = 4.0
    TRIANG_MIN_PARALLAX_DEG = 1.0
    TRIANG_MAX_DIST = 200.0

    # Map
    MAX_MAP_POINTS = 50000

    # Visualization
    TRAJ_CANVAS_SIZE = 400
    MAP_CANVAS_SIZE = 400
    TRAIL_LENGTH = 500             # trajectory trail


# ─────────────────────────── Utility functions ───────────────────────

def make_camera_matrix(w, h, fov_deg=60):
    """Estimate a default camera intrinsic matrix from image dimensions."""
    f = w / (2.0 * np.tan(np.deg2rad(fov_deg / 2.0)))
    K = np.array([
        [f, 0, w / 2.0],
        [0, f, h / 2.0],
        [0, 0, 1.0]
    ], dtype=np.float64)
    return K


def triangulate_points(K, R1, t1, R2, t2, pts1, pts2):
    """Triangulate 3D points from two camera poses and matched 2D points."""
    P1 = K @ np.hstack([R1, t1])
    P2 = K @ np.hstack([R2, t2])
    pts4d = cv2.triangulatePoints(P1, P2, pts1.T, pts2.T)  # 4×N
    pts3d = (pts4d[:3] / pts4d[3:]).T  # N×3
    return pts3d


def filter_triangulated(pts3d, K, R1, t1, R2, t2, pts1_2d, pts2_2d, cfg):
    """Filter triangulated points: positive depth, reprojection error, distance."""
    good = []
    P1 = K @ np.hstack([R1, t1])
    P2 = K @ np.hstack([R2, t2])

    for i, pt3 in enumerate(pts3d):
        # Check positive depth in both cameras
        pc1 = R1 @ pt3 + t1.flatten()
        pc2 = R2 @ pt3 + t2.flatten()
        if pc1[2] <= 0 or pc2[2] <= 0:
            continue

        # Distance check
        if np.linalg.norm(pt3) > cfg.TRIANG_MAX_DIST:
            continue

        # Reprojection error
        p1_h = P1 @ np.append(pt3, 1.0)
        p1_proj = p1_h[:2] / p1_h[2]
        err1 = np.linalg.norm(p1_proj - pts1_2d[i])

        p2_h = P2 @ np.append(pt3, 1.0)
        p2_proj = p2_h[:2] / p2_h[2]
        err2 = np.linalg.norm(p2_proj - pts2_2d[i])

        if err1 < cfg.TRIANG_MAX_REPROJ_ERR and err2 < cfg.TRIANG_MAX_REPROJ_ERR:
            good.append(pt3)

    return np.array(good) if good else np.empty((0, 3))


# ─────────────────────────── SLAM Core ───────────────────────────────

class Frame:
    """Represents a single processed frame."""
    __slots__ = ['idx', 'gray', 'kps', 'des', 'R', 't']

    def __init__(self, idx, gray, kps=None, des=None, R=None, t=None):
        self.idx = idx
        self.gray = gray
        self.kps = kps
        self.des = des
        self.R = R if R is not None else np.eye(3)
        self.t = t if t is not None else np.zeros((3, 1))


class VisualSLAM:
    def __init__(self, K, cfg=None):
        self.K = K
        self.cfg = cfg or SLAMConfig()

        # ORB detector
        self.orb = cv2.ORB_create(
            nfeatures=self.cfg.MAX_FEATURES,
            scaleFactor=self.cfg.ORB_SCALE_FACTOR,
            nlevels=self.cfg.ORB_LEVELS
        )

        # Matcher for ORB (Hamming distance)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        # FAST detector for KLT
        self.fast = cv2.FastFeatureDetector_create(threshold=self.cfg.FAST_THRESHOLD)

        # State
        self.cur_R = np.eye(3)
        self.cur_t = np.zeros((3, 1))
        self.trajectory = []          # list of (x, y, z)
        self.map_points = deque(maxlen=self.cfg.MAX_MAP_POINTS)
        self.keyframes = []

        # KLT state
        self.prev_gray = None
        self.prev_pts = None

        self.frame_idx = 0
        self.initialized = False

    # ── Feature detection ──
    def detect_features_orb(self, gray):
        kps, des = self.orb.detectAndCompute(gray, None)
        return kps, des

    def detect_features_fast(self, gray):
        kps = self.fast.detect(gray, None)
        # Convert to point array
        pts = np.array([kp.pt for kp in kps], dtype=np.float32).reshape(-1, 1, 2)
        return pts

    # ── Feature matching (ORB) ──
    def match_features(self, des1, des2):
        if des1 is None or des2 is None:
            return []
        matches = self.bf.knnMatch(des1, des2, k=2)
        good = []
        for m_pair in matches:
            if len(m_pair) == 2:
                m, n = m_pair
                if m.distance < self.cfg.MATCH_RATIO_THRESH * n.distance:
                    good.append(m)
        return good

    # ── KLT Tracking ──
    def track_klt(self, prev_gray, cur_gray, prev_pts):
        """Track points from prev frame to cur frame using KLT optical flow."""
        if prev_pts is None or len(prev_pts) == 0:
            return None, None, None

        next_pts, status, err = cv2.calcOpticalFlowPyrLK(
            prev_gray, cur_gray, prev_pts, None,
            winSize=self.cfg.KLT_WIN_SIZE,
            maxLevel=self.cfg.KLT_MAX_LEVEL,
            criteria=self.cfg.KLT_CRITERIA
        )

        if next_pts is None:
            return None, None, None

        status = status.flatten()
        good_old = prev_pts[status == 1].reshape(-1, 2)
        good_new = next_pts[status == 1].reshape(-1, 2)

        return good_old, good_new, status

    # ── Pose estimation ──
    def estimate_pose(self, pts1, pts2):
        """Estimate relative pose from point correspondences using essential matrix."""
        if len(pts1) < 8:
            return None, None, None

        E, mask_e = cv2.findEssentialMat(
            pts1, pts2, self.K,
            method=cv2.RANSAC,
            prob=self.cfg.RANSAC_PROB,
            threshold=self.cfg.RANSAC_THRESH
        )

        if E is None:
            return None, None, None

        inliers, R, t, mask_p = cv2.recoverPose(E, pts1, pts2, self.K, mask=mask_e)

        if inliers < self.cfg.KEYFRAME_MIN_INLIERS:
            return None, None, None

        return R, t, mask_e

    # ── Process frame ──
    def process_frame(self, color_frame):
        gray = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
        info = {
            'num_matches': 0,
            'num_inliers': 0,
            'num_map_pts': len(self.map_points),
            'matched_pts1': None,
            'matched_pts2': None,
            'is_keyframe': False,
        }

        if self.cfg.USE_KLT:
            info = self._process_klt(gray, info)
        else:
            info = self._process_orb(gray, info)

        self.trajectory.append(self.cur_t.flatten().copy())
        self.frame_idx += 1
        info['num_map_pts'] = len(self.map_points)
        return info

    def _process_klt(self, gray, info):
        """Process using KLT optical flow tracking."""
        if self.prev_gray is None:
            # First frame: detect and store
            self.prev_gray = gray
            self.prev_pts = self.detect_features_fast(gray)
            self.initialized = False
            return info

        # Track
        old_pts, new_pts, status = self.track_klt(self.prev_gray, gray, self.prev_pts)

        if old_pts is None or len(old_pts) < self.cfg.MIN_MATCHES:
            # Lost tracking — re-detect
            self.prev_gray = gray
            self.prev_pts = self.detect_features_fast(gray)
            return info

        info['num_matches'] = len(old_pts)
        info['matched_pts1'] = old_pts
        info['matched_pts2'] = new_pts

        # Estimate pose
        R, t, mask = self.estimate_pose(old_pts, new_pts)

        if R is not None:
            # Compose global pose: T_cur = T_prev * T_rel
            self.cur_t = self.cur_t + self.cur_R @ t
            self.cur_R = R @ self.cur_R

            info['num_inliers'] = int(mask.sum()) if mask is not None else 0

            # Triangulate new map points from inlier matches
            if mask is not None:
                mask_flat = mask.flatten().astype(bool)
                inlier_old = old_pts[mask_flat]
                inlier_new = new_pts[mask_flat]

                if len(inlier_old) > 10:
                    # Use previous and current pose
                    R_prev = np.eye(3)  # relative frame: prev is identity
                    t_prev = np.zeros((3, 1))
                    pts3d = triangulate_points(
                        self.K, R_prev, t_prev, R, t,
                        inlier_old.astype(np.float64),
                        inlier_new.astype(np.float64)
                    )
                    # Filter
                    good3d = filter_triangulated(
                        pts3d, self.K, R_prev, t_prev, R, t,
                        inlier_old, inlier_new, self.cfg
                    )
                    # Transform to world frame
                    if len(good3d) > 0:
                        # Previous camera's world pose
                        R_w = self.cur_R.T  # approximation
                        t_w = self.cur_t
                        world_pts = (R_w @ good3d.T).T + t_w.flatten()
                        for pt in world_pts[::3]:  # subsample to avoid flooding
                            self.map_points.append(pt)
                        info['is_keyframe'] = True

        # Re-detect if too few points tracked
        num_tracked = len(new_pts)
        if num_tracked < self.cfg.REDETECT_THRESHOLD:
            self.prev_pts = self.detect_features_fast(gray)
        else:
            self.prev_pts = new_pts.reshape(-1, 1, 2)

        self.prev_gray = gray
        return info

    def _process_orb(self, gray, info):
        """Process using ORB detection + BFMatcher."""
        kps, des = self.detect_features_orb(gray)

        if not hasattr(self, '_prev_kps') or self._prev_des is None:
            self._prev_gray = gray
            self._prev_kps = kps
            self._prev_des = des
            return info

        matches = self.match_features(self._prev_des, des)
        info['num_matches'] = len(matches)

        if len(matches) >= self.cfg.MIN_MATCHES:
            pts1 = np.array([self._prev_kps[m.queryIdx].pt for m in matches], dtype=np.float64)
            pts2 = np.array([kps[m.trainIdx].pt for m in matches], dtype=np.float64)
            info['matched_pts1'] = pts1
            info['matched_pts2'] = pts2

            R, t, mask = self.estimate_pose(pts1, pts2)

            if R is not None:
                self.cur_t = self.cur_t + self.cur_R @ t
                self.cur_R = R @ self.cur_R
                info['num_inliers'] = int(mask.sum()) if mask is not None else 0

        self._prev_gray = gray
        self._prev_kps = kps
        self._prev_des = des
        return info


# ─────────────────────────── Visualization ───────────────────────────

class SLAMVisualizer:
    def __init__(self, frame_w, frame_h, cfg=None):
        self.cfg = cfg or SLAMConfig()
        self.frame_w = frame_w
        self.frame_h = frame_h
        self.traj_size = self.cfg.TRAJ_CANVAS_SIZE
        self.map_size = self.cfg.MAP_CANVAS_SIZE

        # Panel layout:
        # [  Original + features  |  Trajectory (top-down XZ)  ]
        # [                       |  3D Map (projected XY)     ]
        self.panel_w = frame_w + self.traj_size
        self.panel_h = max(frame_h, self.traj_size + self.map_size)

        # Trajectory canvas state
        self.traj_canvas = np.zeros((self.traj_size, self.traj_size, 3), dtype=np.uint8)
        self.traj_scale = 2.0       # will auto-adjust
        self.traj_center = np.array([self.traj_size // 2, self.traj_size // 2])

    def draw(self, color_frame, slam, info):
        """Compose the full output frame."""
        vis = color_frame.copy()

        # ── Draw tracked features ──
        if info['matched_pts2'] is not None:
            pts2 = info['matched_pts2']
            pts1 = info['matched_pts1']
            for i in range(min(len(pts2), 500)):
                x2, y2 = int(pts2[i][0]), int(pts2[i][1])
                x1, y1 = int(pts1[i][0]), int(pts1[i][1])
                cv2.circle(vis, (x2, y2), 3, (0, 255, 0), -1)
                cv2.line(vis, (x1, y1), (x2, y2), (0, 200, 200), 1)

        # ── HUD overlay ──
        hud_lines = [
            f"Frame: {slam.frame_idx}",
            f"Matches: {info['num_matches']}",
            f"Inliers: {info['num_inliers']}",
            f"Map pts: {info['num_map_pts']}",
        ]
        for i, line in enumerate(hud_lines):
            cv2.putText(vis, line, (10, 25 + i * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)

        # ── Draw "KEYFRAME" indicator ──
        if info.get('is_keyframe'):
            cv2.putText(vis, "KEYFRAME", (self.frame_w - 140, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

        # ── Trajectory panel (top-down XZ plane) ──
        traj_panel = self._draw_trajectory(slam)

        # ── Map panel (projected 3D points) ──
        map_panel = self._draw_map(slam)

        # ── Compose panels ──
        right_panel = np.zeros((self.panel_h, self.traj_size, 3), dtype=np.uint8)
        right_panel[:self.traj_size, :] = traj_panel
        y_off = self.traj_size
        h_remaining = self.panel_h - y_off
        if h_remaining > 0:
            map_resized = cv2.resize(map_panel, (self.traj_size, min(self.map_size, h_remaining)))
            right_panel[y_off:y_off + map_resized.shape[0], :] = map_resized

        # Left panel (video)
        left_panel = np.zeros((self.panel_h, self.frame_w, 3), dtype=np.uint8)
        left_panel[:self.frame_h, :] = vis

        output = np.hstack([left_panel, right_panel])
        return output

    def _draw_trajectory(self, slam):
        """Draw top-down trajectory (XZ plane)."""
        canvas = np.zeros((self.traj_size, self.traj_size, 3), dtype=np.uint8)

        # Title
        cv2.putText(canvas, "Trajectory (top-down)", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)

        if len(slam.trajectory) < 2:
            return canvas

        traj = np.array(slam.trajectory)
        xs = traj[:, 0]  # X
        zs = traj[:, 2]  # Z (forward)

        # Auto-scale to fit canvas with margin
        margin = 40
        usable = self.traj_size - 2 * margin
        x_range = max(xs.max() - xs.min(), 1e-3)
        z_range = max(zs.max() - zs.min(), 1e-3)
        scale = usable / max(x_range, z_range)

        cx = self.traj_size // 2
        cy = self.traj_size // 2
        x_mid = (xs.max() + xs.min()) / 2
        z_mid = (zs.max() + zs.min()) / 2

        # Draw path
        trail = slam.trajectory[-self.cfg.TRAIL_LENGTH:]
        trail_arr = np.array(trail)
        for i in range(1, len(trail_arr)):
            x1 = int(cx + (trail_arr[i-1, 0] - x_mid) * scale)
            y1 = int(cy + (trail_arr[i-1, 2] - z_mid) * scale)
            x2 = int(cx + (trail_arr[i, 0] - x_mid) * scale)
            y2 = int(cy + (trail_arr[i, 2] - z_mid) * scale)

            # Color gradient: older = darker blue, newer = bright green
            alpha = i / len(trail_arr)
            color = (
                int(50 * (1 - alpha)),
                int(100 + 155 * alpha),
                int(200 * (1 - alpha))
            )
            cv2.line(canvas, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)

        # Current position
        cur_x = int(cx + (xs[-1] - x_mid) * scale)
        cur_y = int(cy + (zs[-1] - z_mid) * scale)
        cv2.circle(canvas, (cur_x, cur_y), 5, (0, 0, 255), -1)

        # Start position
        start_x = int(cx + (xs[0] - x_mid) * scale)
        start_y = int(cy + (zs[0] - z_mid) * scale)
        cv2.circle(canvas, (start_x, start_y), 5, (255, 255, 0), -1)

        # Scale bar
        cv2.putText(canvas, f"Scale: {1.0/scale:.2f} m/px", (10, self.traj_size - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1, cv2.LINE_AA)

        return canvas

    def _draw_map(self, slam):
        """Draw sparse 3D map points (projected to XY view)."""
        canvas = np.zeros((self.map_size, self.map_size, 3), dtype=np.uint8)

        cv2.putText(canvas, "3D Map Points (top view)", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1, cv2.LINE_AA)

        if len(slam.map_points) < 3:
            return canvas

        pts = np.array(list(slam.map_points))
        xs = pts[:, 0]
        zs = pts[:, 2]

        margin = 30
        usable = self.map_size - 2 * margin
        x_range = max(xs.max() - xs.min(), 1e-3)
        z_range = max(zs.max() - zs.min(), 1e-3)
        scale = usable / max(x_range, z_range)

        cx = self.map_size // 2
        cy = self.map_size // 2
        x_mid = (xs.max() + xs.min()) / 2
        z_mid = (zs.max() + zs.min()) / 2

        for pt in pts:
            px = int(cx + (pt[0] - x_mid) * scale)
            py = int(cy + (pt[2] - z_mid) * scale)
            if 0 <= px < self.map_size and 0 <= py < self.map_size:
                # Color by height (Y)
                h_norm = np.clip((pt[1] - pts[:, 1].min()) / max(pts[:, 1].max() - pts[:, 1].min(), 1e-3), 0, 1)
                color = (
                    int(255 * (1 - h_norm)),
                    int(100 + 155 * h_norm),
                    int(255 * h_norm)
                )
                cv2.circle(canvas, (px, py), 1, color, -1)

        # Draw camera position
        if len(slam.trajectory) > 0:
            cur = slam.trajectory[-1]
            cam_x = int(cx + (cur[0] - x_mid) * scale)
            cam_y = int(cy + (cur[2] - z_mid) * scale)
            cv2.drawMarker(canvas, (cam_x, cam_y), (0, 0, 255),
                           cv2.MARKER_DIAMOND, 10, 2)

        cv2.putText(canvas, f"{len(slam.map_points)} pts", (10, self.map_size - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1, cv2.LINE_AA)

        return canvas


# ─────────────────────────── Main Pipeline ───────────────────────────

def run_slam(input_path, output_path):
    cap = cv2.VideoCapture(input_path)
    if not cap.isOpened():
        print(f"ERROR: Cannot open video: {input_path}")
        sys.exit(1)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    # Auto-downscale to 480p if larger
    max_h = 480
    if h > max_h:
        scale_factor = max_h / h
        w = int(w * scale_factor)
        h = max_h
        print(f"Downscaling to {w}x{h} (480p)")

    print(f"Input : {input_path}")
    print(f"Size  : {w}x{h} @ {fps:.1f} fps, {total_frames} frames")
    print(f"Output: {output_path}")

    # Camera intrinsics (estimated from frame size, ~60° FOV)
    K = make_camera_matrix(w, h, fov_deg=60)
    print(f"Estimated K:\n{K}")

    cfg = SLAMConfig()
    slam = VisualSLAM(K, cfg)
    viz = SLAMVisualizer(w, h, cfg)

    # Output video
    out_w = viz.panel_w
    out_h = viz.panel_h
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(output_path, fourcc, fps, (out_w, out_h))

    frame_num = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        # Resize if needed
        if frame.shape[1] != w or frame.shape[0] != h:
            frame = cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA)

        info = slam.process_frame(frame)
        output_frame = viz.draw(frame, slam, info)
        writer.write(output_frame)

        frame_num += 1
        if frame_num % 50 == 0 or frame_num == 1:
            pct = frame_num / max(total_frames, 1) * 100
            print(f"  [{frame_num}/{total_frames}] {pct:.0f}%  |  "
                  f"matches={info['num_matches']}  inliers={info['num_inliers']}  "
                  f"map_pts={info['num_map_pts']}")

    cap.release()
    writer.release()
    print(f"\nDone! Processed {frame_num} frames.")
    print(f"Trajectory length: {len(slam.trajectory)} poses")
    print(f"Map points: {len(slam.map_points)}")
    print(f"Output saved to: {output_path}")

    # ── Export point cloud and trajectory for 3D reconstruction ──
    import os
    base = os.path.splitext(output_path)[0]
    ply_path = base + "_map.ply"
    traj_path = base + "_trajectory.csv"

    pts = np.array(list(slam.map_points))
    if len(pts) > 0:
        with open(ply_path, 'w') as f:
            f.write(f"ply\nformat ascii 1.0\nelement vertex {len(pts)}\n")
            f.write("property float x\nproperty float y\nproperty float z\nend_header\n")
            for p in pts:
                f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
        print(f"Point cloud saved: {ply_path} ({len(pts)} points)")

    traj = np.array(slam.trajectory)
    if len(traj) > 0:
        np.savetxt(traj_path, traj, delimiter=',', header='x,y,z', comments='')
        print(f"Trajectory saved: {traj_path} ({len(traj)} poses)")

    return ply_path, traj_path


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 visual_slam.py <input.mp4> [output.mp4]")
        sys.exit(1)

    input_path = sys.argv[1]
    output_path = sys.argv[2] if len(sys.argv) > 2 else "slam_output.mp4"
    run_slam(input_path, output_path)