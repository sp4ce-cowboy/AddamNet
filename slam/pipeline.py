#!/usr/bin/env python3
"""
Full Pipeline: Video → Visual SLAM → 3D Reconstruction → Interactive Viewer
=============================================================================

One command to go from an input .mp4 to an interactive 3D viewer in the browser.

Usage:
    python3 pipeline.py input.mp4 [--output-dir results/]

Outputs:
    results/slam_output.mp4       - SLAM visualization video
    results/slam_output_map.ply   - sparse 3D point cloud
    results/slam_output_trajectory.csv - camera trajectory
    results/slam_viewer.html      - interactive 3D browser viewer
"""

import sys
import os
import argparse


def main():
    parser = argparse.ArgumentParser(
        description="Video → SLAM → 3D Reconstruction → Interactive Viewer"
    )
    parser.add_argument("input_video", help="Input .mp4 video file")
    parser.add_argument("--output-dir", "-d", default=".", help="Output directory (default: current)")
    parser.add_argument("--no-mesh", action="store_true", help="Skip mesh, show point cloud only")
    parser.add_argument("--alpha", type=float, default=None, help="Alpha for mesh reconstruction")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    video_out = os.path.join(args.output_dir, "slam_output.mp4")
    viewer_out = os.path.join(args.output_dir, "slam_viewer.html")

    # ── Step 1: Visual SLAM ──
    print("=" * 60)
    print("STEP 1: Visual SLAM")
    print("=" * 60)
    from visual_slam import run_slam
    ply_path, traj_path = run_slam(args.input_video, video_out)

    # ── Step 2: 3D Reconstruction + Viewer ──
    print()
    print("=" * 60)
    print("STEP 2: 3D Reconstruction & Viewer")
    print("=" * 60)
    from reconstruct_viewer import (
        read_ply, read_trajectory,
        statistical_outlier_removal, estimate_normals,
        reconstruct_surface, generate_html_viewer
    )

    pts = read_ply(ply_path)
    print(f"Loaded {len(pts)} map points")

    trajectory = read_trajectory(traj_path)
    if trajectory is not None:
        print(f"Loaded {len(trajectory)} trajectory poses")

    print("Cleaning outliers...")
    pts_clean, _ = statistical_outlier_removal(pts, k=20, std_ratio=2.0)
    print(f"  {len(pts_clean)} points after outlier removal")

    print("Estimating normals...")
    normals = estimate_normals(pts_clean, k=15)

    import numpy as np
    faces = np.empty((0, 3), dtype=int)
    if not args.no_mesh:
        print("Reconstructing surface mesh...")
        faces = reconstruct_surface(pts_clean, normals, alpha=args.alpha)
        print(f"  {len(faces)} mesh faces")

    print(f"Generating viewer → {viewer_out}")
    html = generate_html_viewer(pts_clean, normals, faces, trajectory,
                                 title="SLAM 3D Reconstruction")
    with open(viewer_out, 'w') as f:
        f.write(html)

    # ── Summary ──
    print()
    print("=" * 60)
    print("DONE! Outputs:")
    print("=" * 60)
    for path in [video_out, ply_path, traj_path, viewer_out]:
        if os.path.exists(path):
            size = os.path.getsize(path) / 1024
            unit = "KB"
            if size > 1024:
                size /= 1024
                unit = "MB"
            print(f"  {path:40s}  {size:.1f} {unit}")

    print(f"\nOpen {viewer_out} in a browser to explore your 3D reconstruction!")


if __name__ == "__main__":
    main()