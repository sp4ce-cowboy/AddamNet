#!/usr/bin/env python3
"""
3D Reconstruction & Interactive Viewer from Visual SLAM Output
================================================================
Reads a sparse point cloud (.ply) and optional trajectory (.csv) from the
visual SLAM pipeline, performs:
  1. Statistical outlier removal
  2. Surface reconstruction (Alpha-shape via Delaunay + edge filtering)
  3. Normal estimation
  4. Exports a standalone interactive HTML viewer (Three.js)

Usage:
    python3 reconstruct_viewer.py map.ply [trajectory.csv] [--output viewer.html]

The output HTML is fully self-contained (loads Three.js from CDN) and can
be opened in any modern browser for interactive 3D exploration with:
  - Orbit / pan / zoom controls
  - Toggle: point cloud, mesh, trajectory, wireframe, normals
  - Color modes: height, depth, normals, solid
  - Adjustable point size
  - Screenshot button
"""

import sys
import argparse
import json
import numpy as np
from scipy.spatial import Delaunay, KDTree


# ─────────────────────── PLY I/O ─────────────────────────────────────

def read_ply(path):
    """Read an ASCII .ply file → Nx3 numpy array."""
    with open(path, 'r') as f:
        lines = f.readlines()

    header_end = 0
    n_verts = 0
    for i, line in enumerate(lines):
        if line.strip().startswith('element vertex'):
            n_verts = int(line.strip().split()[-1])
        if line.strip() == 'end_header':
            header_end = i + 1
            break

    pts = []
    for line in lines[header_end:header_end + n_verts]:
        vals = line.strip().split()
        if len(vals) >= 3:
            pts.append([float(vals[0]), float(vals[1]), float(vals[2])])

    return np.array(pts, dtype=np.float64)


def read_trajectory(path):
    """Read trajectory CSV (x,y,z per line)."""
    try:
        data = np.loadtxt(path, delimiter=',', skiprows=1)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        return data[:, :3]
    except Exception:
        return None


# ─────────────────────── Point Cloud Processing ─────────────────────

def statistical_outlier_removal(pts, k=20, std_ratio=2.0):
    """Remove points whose mean distance to k-nearest neighbors exceeds threshold."""
    if len(pts) < k + 1:
        return pts, np.ones(len(pts), dtype=bool)

    tree = KDTree(pts)
    dists, _ = tree.query(pts, k=k + 1)  # +1 because self is included
    mean_dists = dists[:, 1:].mean(axis=1)  # exclude self

    global_mean = mean_dists.mean()
    global_std = mean_dists.std()
    threshold = global_mean + std_ratio * global_std

    mask = mean_dists < threshold
    return pts[mask], mask


def estimate_normals(pts, k=15):
    """Estimate per-vertex normals using PCA on k-nearest neighbors."""
    tree = KDTree(pts)
    _, idx = tree.query(pts, k=k)
    normals = np.zeros_like(pts)

    for i in range(len(pts)):
        neighbors = pts[idx[i]]
        centered = neighbors - neighbors.mean(axis=0)
        cov = centered.T @ centered
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        # Normal is eigenvector with smallest eigenvalue
        normals[i] = eigenvectors[:, 0]

    # Orient normals consistently (point away from centroid)
    centroid = pts.mean(axis=0)
    for i in range(len(normals)):
        if np.dot(normals[i], pts[i] - centroid) < 0:
            normals[i] *= -1

    # Normalize
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    norms[norms < 1e-10] = 1
    normals /= norms

    return normals


# ─────────────────────── Surface Reconstruction ─────────────────────

def alpha_shape_3d(pts, alpha=None):
    """
    Compute alpha-shape triangulation of a 3D point cloud.
    Returns triangle faces as Nx3 index array.

    If alpha is None, it is estimated automatically from point spacing.
    """
    if len(pts) < 4:
        return np.empty((0, 3), dtype=int)

    try:
        tri = Delaunay(pts)
    except Exception:
        return np.empty((0, 3), dtype=int)

    # Auto-estimate alpha from nearest-neighbor statistics
    if alpha is None:
        tree = KDTree(pts)
        dists, _ = tree.query(pts, k=6)
        # Use the 5th-NN distance at the 90th percentile — generous enough for sparse clouds
        alpha = np.percentile(dists[:, 5], 90) * 2.5

    print(f"  Alpha value: {alpha:.3f}")

    # Collect boundary faces (appear in exactly one tetrahedron)
    face_count = {}
    for simplex in tri.simplices:
        for combo in [(0,1,2), (0,1,3), (0,2,3), (1,2,3)]:
            face = tuple(sorted(simplex[list(combo)]))
            face_count[face] = face_count.get(face, 0) + 1

    surface_faces = []
    for face, count in face_count.items():
        if count == 1:
            p0, p1, p2 = pts[face[0]], pts[face[1]], pts[face[2]]
            e_max = max(np.linalg.norm(p1-p0), np.linalg.norm(p2-p1), np.linalg.norm(p0-p2))
            if e_max < alpha:
                surface_faces.append(face)

    # Also include internal faces with small edges (fills in thin structures)
    if len(surface_faces) < 50:
        tight_alpha = alpha * 0.7
        for face, count in face_count.items():
            if count == 2:
                p0, p1, p2 = pts[face[0]], pts[face[1]], pts[face[2]]
                e_max = max(np.linalg.norm(p1-p0), np.linalg.norm(p2-p1), np.linalg.norm(p0-p2))
                if e_max < tight_alpha:
                    surface_faces.append(face)

    return np.array(surface_faces, dtype=int) if surface_faces else np.empty((0, 3), dtype=int)


def knn_mesh(pts, normals, k=8):
    """
    Build a mesh by connecting each point to its k-nearest neighbors via
    triangles.  Works well for sparse, roughly-uniformly-distributed clouds
    where Delaunay alpha shapes produce too few faces.
    """
    tree = KDTree(pts)
    _, indices = tree.query(pts, k=k + 1)  # +1 for self

    face_set = set()
    for i in range(len(pts)):
        neighbors = indices[i, 1:]  # skip self
        for a_idx in range(len(neighbors)):
            for b_idx in range(a_idx + 1, len(neighbors)):
                a, b = neighbors[a_idx], neighbors[b_idx]
                # Only add triangle if all three are mutual near-neighbors
                if a in indices[b, 1:] or b in indices[a, 1:]:
                    face = tuple(sorted([i, int(a), int(b)]))
                    face_set.add(face)

    if not face_set:
        return np.empty((0, 3), dtype=int)

    faces = np.array(list(face_set), dtype=int)
    return faces


def reconstruct_surface(pts, normals, alpha=None):
    """
    Reconstruct surface mesh using alpha shape with kNN fallback.
    Also filters for normal consistency.
    """
    # Try alpha shape first
    faces = alpha_shape_3d(pts, alpha=alpha)
    print(f"  Alpha shape: {len(faces)} faces")

    # If too few faces, augment with kNN mesh
    if len(faces) < max(50, len(pts) // 20):
        print("  Sparse result — augmenting with kNN mesh...")
        knn_faces = knn_mesh(pts, normals, k=10)
        print(f"  kNN mesh: {len(knn_faces)} faces")
        if len(knn_faces) > 0:
            if len(faces) > 0:
                combined = set(map(tuple, faces.tolist())) | set(map(tuple, knn_faces.tolist()))
                faces = np.array(list(combined), dtype=int)
            else:
                faces = knn_faces

    if len(faces) == 0:
        return faces

    # Filter by normal consistency and degenerate triangles
    good_faces = []
    for face in faces:
        v0, v1, v2 = pts[face[0]], pts[face[1]], pts[face[2]]
        edge1 = v1 - v0
        edge2 = v2 - v0
        face_normal = np.cross(edge1, edge2)
        fn_len = np.linalg.norm(face_normal)
        if fn_len < 1e-10:
            continue  # degenerate
        face_normal /= fn_len

        # Orient face to match vertex normals
        avg_vn = (normals[face[0]] + normals[face[1]] + normals[face[2]]) / 3.0
        if np.dot(face_normal, avg_vn) < 0:
            face = (face[0], face[2], face[1])

        good_faces.append(face)

    return np.array(good_faces, dtype=int) if good_faces else np.empty((0, 3), dtype=int)


# ─────────────────────── HTML Viewer Generation ──────────────────────

def generate_html_viewer(pts, normals, faces, trajectory=None, title="SLAM 3D Viewer"):
    """Generate a self-contained HTML file with Three.js interactive viewer."""

    # Center and scale the data for viewing
    centroid = pts.mean(axis=0)
    pts_centered = pts - centroid
    max_extent = np.abs(pts_centered).max()
    if max_extent < 1e-6:
        max_extent = 1.0
    scale = 50.0 / max_extent
    pts_scaled = pts_centered * scale

    # Prepare trajectory
    traj_json = "[]"
    if trajectory is not None and len(trajectory) > 0:
        traj_centered = (trajectory - centroid) * scale
        # Subsample trajectory for performance
        step = max(1, len(traj_centered) // 1000)
        traj_sub = traj_centered[::step].tolist()
        traj_json = json.dumps([[round(p[0], 3), round(p[1], 3), round(p[2], 3)] for p in traj_sub])

    # Prepare points (subsample if huge)
    max_render_pts = 50000
    if len(pts_scaled) > max_render_pts:
        idx = np.random.choice(len(pts_scaled), max_render_pts, replace=False)
        pts_render = pts_scaled[idx]
        normals_render = normals[idx]
    else:
        pts_render = pts_scaled
        normals_render = normals

    # Compute per-vertex colors (height-based)
    y_vals = pts_render[:, 1]
    y_min, y_max = y_vals.min(), y_vals.max()
    y_range = y_max - y_min if (y_max - y_min) > 1e-6 else 1.0

    pts_list = [[round(p[0], 3), round(p[1], 3), round(p[2], 3)] for p in pts_render]
    normals_list = [[round(n[0], 4), round(n[1], 4), round(n[2], 4)] for n in normals_render]

    pts_json = json.dumps(pts_list)
    normals_json = json.dumps(normals_list)

    # Prepare faces (subsample if huge)
    max_render_faces = 100000
    if len(faces) > max_render_faces:
        idx = np.random.choice(len(faces), max_render_faces, replace=False)
        faces_render = faces[idx]
    else:
        faces_render = faces

    faces_json = json.dumps(faces_render.tolist()) if len(faces_render) > 0 else "[]"

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>{title}</title>
<style>
  * {{ margin: 0; padding: 0; box-sizing: border-box; }}
  body {{ background: #1a1a2e; overflow: hidden; font-family: 'Segoe UI', system-ui, sans-serif; color: #e0e0e0; }}
  #canvas-container {{ width: 100vw; height: 100vh; }}
  canvas {{ display: block; }}

  /* Control Panel */
  #controls {{
    position: fixed; top: 16px; left: 16px;
    background: rgba(15, 15, 35, 0.92); border: 1px solid rgba(100, 140, 255, 0.25);
    border-radius: 12px; padding: 18px 20px; min-width: 240px;
    backdrop-filter: blur(12px); box-shadow: 0 8px 32px rgba(0,0,0,0.5);
    z-index: 100; max-height: 90vh; overflow-y: auto;
  }}
  #controls h2 {{
    font-size: 15px; font-weight: 600; margin-bottom: 14px;
    color: #7aa2f7; letter-spacing: 0.5px;
  }}
  .ctrl-group {{
    margin-bottom: 12px; padding-bottom: 10px;
    border-bottom: 1px solid rgba(255,255,255,0.06);
  }}
  .ctrl-group:last-child {{ border-bottom: none; margin-bottom: 0; }}
  .ctrl-group label {{
    display: flex; align-items: center; gap: 8px;
    font-size: 13px; cursor: pointer; padding: 3px 0;
    transition: color 0.15s;
  }}
  .ctrl-group label:hover {{ color: #7aa2f7; }}
  .ctrl-group label input[type="checkbox"] {{ accent-color: #7aa2f7; }}
  .ctrl-group label input[type="range"] {{
    flex: 1; accent-color: #7aa2f7; height: 4px;
  }}
  .ctrl-group .label-text {{ flex: 1; }}
  .ctrl-group .value-text {{ font-size: 11px; color: #888; min-width: 30px; text-align: right; }}

  select {{
    width: 100%; padding: 5px 8px; border-radius: 6px; border: 1px solid rgba(100,140,255,0.3);
    background: rgba(30, 30, 60, 0.8); color: #e0e0e0; font-size: 12px;
    cursor: pointer; outline: none;
  }}
  select:focus {{ border-color: #7aa2f7; }}

  button.ctrl-btn {{
    width: 100%; padding: 8px; border-radius: 6px; border: 1px solid rgba(100,140,255,0.3);
    background: rgba(40, 50, 90, 0.6); color: #c0c8e8; font-size: 12px;
    cursor: pointer; transition: all 0.15s; margin-top: 4px;
  }}
  button.ctrl-btn:hover {{ background: rgba(60, 80, 140, 0.7); color: #fff; }}

  /* Stats Panel */
  #stats {{
    position: fixed; top: 16px; right: 16px;
    background: rgba(15, 15, 35, 0.85); border: 1px solid rgba(100, 140, 255, 0.15);
    border-radius: 10px; padding: 14px 18px;
    font-size: 12px; line-height: 1.8; z-index: 100;
    backdrop-filter: blur(8px);
  }}
  #stats span {{ color: #7aa2f7; font-weight: 600; }}

  /* Instructions */
  #help {{
    position: fixed; bottom: 16px; left: 50%; transform: translateX(-50%);
    background: rgba(15, 15, 35, 0.75); border-radius: 8px;
    padding: 8px 20px; font-size: 11px; color: #888;
    z-index: 100; white-space: nowrap;
  }}

  /* Axis labels */
  .axis-label {{
    position: fixed; font-size: 11px; font-weight: 700; z-index: 50;
    text-shadow: 0 0 8px rgba(0,0,0,0.8);
  }}
</style>
</head>
<body>
<div id="canvas-container"></div>

<div id="controls">
  <h2>&#x1f30d; SLAM 3D Viewer</h2>

  <div class="ctrl-group">
    <label><input type="checkbox" id="chk-points" checked><span class="label-text">Point Cloud</span></label>
    <label><input type="checkbox" id="chk-mesh" checked><span class="label-text">Mesh Surface</span></label>
    <label><input type="checkbox" id="chk-wireframe"><span class="label-text">Wireframe</span></label>
    <label><input type="checkbox" id="chk-trajectory" checked><span class="label-text">Camera Trajectory</span></label>
    <label><input type="checkbox" id="chk-axes" checked><span class="label-text">Axes</span></label>
  </div>

  <div class="ctrl-group">
    <label>
      <span class="label-text">Point Size</span>
      <input type="range" id="slider-ptsize" min="0.5" max="8" step="0.5" value="2.5">
      <span class="value-text" id="val-ptsize">2.5</span>
    </label>
    <label>
      <span class="label-text">Mesh Opacity</span>
      <input type="range" id="slider-opacity" min="0" max="1" step="0.05" value="0.7">
      <span class="value-text" id="val-opacity">0.7</span>
    </label>
  </div>

  <div class="ctrl-group">
    <label><span class="label-text">Color Mode</span></label>
    <select id="sel-colormode">
      <option value="height" selected>Height (Y)</option>
      <option value="depth">Depth (Z)</option>
      <option value="normals">Normals</option>
      <option value="solid">Solid</option>
    </select>
  </div>

  <div class="ctrl-group">
    <button class="ctrl-btn" id="btn-reset">Reset Camera</button>
    <button class="ctrl-btn" id="btn-screenshot">Screenshot (PNG)</button>
    <button class="ctrl-btn" id="btn-top">Top View</button>
    <button class="ctrl-btn" id="btn-front">Front View</button>
  </div>
</div>

<div id="stats">
  Points: <span id="stat-pts">0</span><br>
  Faces: <span id="stat-faces">0</span><br>
  Trajectory: <span id="stat-traj">0</span> poses
</div>

<div id="help">
  Left-drag: orbit &nbsp;|&nbsp; Right-drag: pan &nbsp;|&nbsp; Scroll: zoom &nbsp;|&nbsp; Double-click: center
</div>

<!-- Three.js from CDN -->
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script>
// ─── Data ───
const POINTS = {pts_json};
const NORMALS = {normals_json};
const FACES = {faces_json};
const TRAJECTORY = {traj_json};

document.getElementById('stat-pts').textContent = POINTS.length;
document.getElementById('stat-faces').textContent = FACES.length;
document.getElementById('stat-traj').textContent = TRAJECTORY.length;

// ─── Scene Setup ───
const container = document.getElementById('canvas-container');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1a2e);
scene.fog = new THREE.FogExp2(0x1a1a2e, 0.003);

const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 2000);
camera.position.set(80, 60, 80);
camera.lookAt(0, 0, 0);

const renderer = new THREE.WebGLRenderer({{ antialias: true, preserveDrawingBuffer: true }});
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
container.appendChild(renderer.domElement);

// ─── Lights ───
const ambientLight = new THREE.AmbientLight(0x404060, 0.6);
scene.add(ambientLight);
const dirLight1 = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight1.position.set(50, 80, 50);
scene.add(dirLight1);
const dirLight2 = new THREE.DirectionalLight(0x8888ff, 0.3);
dirLight2.position.set(-30, -20, -40);
scene.add(dirLight2);

// ─── Orbit Controls (inline minimal implementation) ───
class SimpleOrbitControls {{
  constructor(camera, domElement) {{
    this.camera = camera;
    this.dom = domElement;
    this.target = new THREE.Vector3(0, 0, 0);
    this.spherical = new THREE.Spherical();
    this.sphericalDelta = new THREE.Spherical();
    this.panOffset = new THREE.Vector3();
    this.rotateSpeed = 0.8;
    this.panSpeed = 0.5;
    this.zoomSpeed = 1.2;
    this.isRotating = false;
    this.isPanning = false;
    this.prevMouse = {{ x: 0, y: 0 }};

    this._updateSpherical();

    domElement.addEventListener('mousedown', e => this._onMouseDown(e));
    domElement.addEventListener('mousemove', e => this._onMouseMove(e));
    domElement.addEventListener('mouseup', () => this._onMouseUp());
    domElement.addEventListener('wheel', e => this._onWheel(e), {{ passive: false }});
    domElement.addEventListener('contextmenu', e => e.preventDefault());
    domElement.addEventListener('dblclick', e => this._onDblClick(e));

    // Touch support
    domElement.addEventListener('touchstart', e => this._onTouchStart(e), {{ passive: false }});
    domElement.addEventListener('touchmove', e => this._onTouchMove(e), {{ passive: false }});
    domElement.addEventListener('touchend', () => this._onMouseUp());
  }}

  _updateSpherical() {{
    const offset = this.camera.position.clone().sub(this.target);
    this.spherical.setFromVector3(offset);
  }}

  _onMouseDown(e) {{
    if (e.button === 0) this.isRotating = true;
    if (e.button === 2) this.isPanning = true;
    this.prevMouse = {{ x: e.clientX, y: e.clientY }};
  }}

  _onMouseMove(e) {{
    const dx = e.clientX - this.prevMouse.x;
    const dy = e.clientY - this.prevMouse.y;
    this.prevMouse = {{ x: e.clientX, y: e.clientY }};

    if (this.isRotating) {{
      this.spherical.theta -= dx * this.rotateSpeed * 0.005;
      this.spherical.phi -= dy * this.rotateSpeed * 0.005;
      this.spherical.phi = Math.max(0.01, Math.min(Math.PI - 0.01, this.spherical.phi));
      this._applySpherical();
    }}
    if (this.isPanning) {{
      const dist = this.spherical.radius;
      const panX = -dx * this.panSpeed * dist * 0.002;
      const panY = dy * this.panSpeed * dist * 0.002;
      const right = new THREE.Vector3().setFromMatrixColumn(this.camera.matrix, 0);
      const up = new THREE.Vector3().setFromMatrixColumn(this.camera.matrix, 1);
      this.target.add(right.multiplyScalar(panX));
      this.target.add(up.multiplyScalar(panY));
      this._applySpherical();
    }}
  }}

  _onMouseUp() {{ this.isRotating = false; this.isPanning = false; }}

  _onWheel(e) {{
    e.preventDefault();
    const factor = e.deltaY > 0 ? 1.1 : 0.9;
    this.spherical.radius *= factor;
    this.spherical.radius = Math.max(1, Math.min(500, this.spherical.radius));
    this._applySpherical();
  }}

  _onDblClick(e) {{
    this.target.set(0, 0, 0);
    this._applySpherical();
  }}

  _onTouchStart(e) {{
    if (e.touches.length === 1) {{
      this.isRotating = true;
      this.prevMouse = {{ x: e.touches[0].clientX, y: e.touches[0].clientY }};
    }} else if (e.touches.length === 2) {{
      this.isPanning = true;
      const cx = (e.touches[0].clientX + e.touches[1].clientX) / 2;
      const cy = (e.touches[0].clientY + e.touches[1].clientY) / 2;
      this.prevMouse = {{ x: cx, y: cy }};
      this._pinchDist = Math.hypot(
        e.touches[0].clientX - e.touches[1].clientX,
        e.touches[0].clientY - e.touches[1].clientY
      );
    }}
    e.preventDefault();
  }}

  _onTouchMove(e) {{
    if (e.touches.length === 1 && this.isRotating) {{
      const dx = e.touches[0].clientX - this.prevMouse.x;
      const dy = e.touches[0].clientY - this.prevMouse.y;
      this.prevMouse = {{ x: e.touches[0].clientX, y: e.touches[0].clientY }};
      this.spherical.theta -= dx * this.rotateSpeed * 0.005;
      this.spherical.phi -= dy * this.rotateSpeed * 0.005;
      this.spherical.phi = Math.max(0.01, Math.min(Math.PI - 0.01, this.spherical.phi));
      this._applySpherical();
    }} else if (e.touches.length === 2) {{
      const cx = (e.touches[0].clientX + e.touches[1].clientX) / 2;
      const cy = (e.touches[0].clientY + e.touches[1].clientY) / 2;
      const dist = Math.hypot(
        e.touches[0].clientX - e.touches[1].clientX,
        e.touches[0].clientY - e.touches[1].clientY
      );
      // Pinch zoom
      if (this._pinchDist) {{
        const factor = this._pinchDist / dist;
        this.spherical.radius *= factor;
        this.spherical.radius = Math.max(1, Math.min(500, this.spherical.radius));
      }}
      // Pan
      const dx = cx - this.prevMouse.x;
      const dy = cy - this.prevMouse.y;
      const panScale = this.spherical.radius * 0.002;
      const right = new THREE.Vector3().setFromMatrixColumn(camera.matrix, 0);
      const up = new THREE.Vector3().setFromMatrixColumn(camera.matrix, 1);
      this.target.add(right.multiplyScalar(-dx * panScale));
      this.target.add(up.multiplyScalar(dy * panScale));

      this.prevMouse = {{ x: cx, y: cy }};
      this._pinchDist = dist;
      this._applySpherical();
    }}
    e.preventDefault();
  }}

  _applySpherical() {{
    const offset = new THREE.Vector3().setFromSpherical(this.spherical);
    this.camera.position.copy(this.target).add(offset);
    this.camera.lookAt(this.target);
  }}

  setPosition(x, y, z) {{
    this.camera.position.set(x, y, z);
    this._updateSpherical();
    this._applySpherical();
  }}
}}

const controls = new SimpleOrbitControls(camera, renderer.domElement);

// ─── Build Point Cloud ───
function computeColors(points, mode) {{
  const colors = new Float32Array(points.length * 3);
  const n = points.length / 3;

  let vals = new Float32Array(n);
  if (mode === 'height') {{
    for (let i = 0; i < n; i++) vals[i] = points[i * 3 + 1]; // Y
  }} else if (mode === 'depth') {{
    for (let i = 0; i < n; i++) vals[i] = points[i * 3 + 2]; // Z
  }}

  if (mode === 'height' || mode === 'depth') {{
    let vmin = Infinity, vmax = -Infinity;
    for (let i = 0; i < n; i++) {{ vmin = Math.min(vmin, vals[i]); vmax = Math.max(vmax, vals[i]); }}
    const vrange = (vmax - vmin) || 1;
    for (let i = 0; i < n; i++) {{
      const t = (vals[i] - vmin) / vrange;
      // Viridis-like colormap
      colors[i * 3] = Math.min(1, Math.max(0, 0.267 + 0.7 * t + 0.5 * Math.sin(t * 3.14)));
      colors[i * 3 + 1] = Math.min(1, Math.max(0, 0.004 + 0.87 * t));
      colors[i * 3 + 2] = Math.min(1, Math.max(0, 0.329 + 0.45 * Math.sin((1 - t) * 3.14)));
    }}
  }} else if (mode === 'normals') {{
    for (let i = 0; i < n; i++) {{
      const ni = i < NORMALS.length ? i : 0;
      colors[i * 3] = Math.abs(NORMALS[ni][0]);
      colors[i * 3 + 1] = Math.abs(NORMALS[ni][1]);
      colors[i * 3 + 2] = Math.abs(NORMALS[ni][2]);
    }}
  }} else {{
    for (let i = 0; i < n; i++) {{
      colors[i * 3] = 0.45;
      colors[i * 3 + 1] = 0.65;
      colors[i * 3 + 2] = 0.95;
    }}
  }}
  return colors;
}}

// Point cloud
const pointsFlat = new Float32Array(POINTS.length * 3);
for (let i = 0; i < POINTS.length; i++) {{
  pointsFlat[i * 3] = POINTS[i][0];
  pointsFlat[i * 3 + 1] = POINTS[i][1];
  pointsFlat[i * 3 + 2] = POINTS[i][2];
}}

const pcGeom = new THREE.BufferGeometry();
pcGeom.setAttribute('position', new THREE.BufferAttribute(pointsFlat, 3));
pcGeom.setAttribute('color', new THREE.BufferAttribute(computeColors(pointsFlat, 'height'), 3));

const pcMat = new THREE.PointsMaterial({{
  size: 2.5, vertexColors: true, sizeAttenuation: true,
  transparent: true, opacity: 0.9
}});
const pointCloud = new THREE.Points(pcGeom, pcMat);
scene.add(pointCloud);

// ─── Build Mesh ───
let meshObj = null;
let wireframeObj = null;

if (FACES.length > 0) {{
  const meshGeom = new THREE.BufferGeometry();
  const meshPositions = new Float32Array(FACES.length * 9);
  const meshNorms = new Float32Array(FACES.length * 9);
  const meshColors = new Float32Array(FACES.length * 9);

  const pcColors = pcGeom.attributes.color.array;

  for (let i = 0; i < FACES.length; i++) {{
    for (let j = 0; j < 3; j++) {{
      const vi = FACES[i][j];
      const pi = vi < POINTS.length ? vi : 0;
      meshPositions[i * 9 + j * 3] = POINTS[pi][0];
      meshPositions[i * 9 + j * 3 + 1] = POINTS[pi][1];
      meshPositions[i * 9 + j * 3 + 2] = POINTS[pi][2];

      const ni = pi < NORMALS.length ? pi : 0;
      meshNorms[i * 9 + j * 3] = NORMALS[ni][0];
      meshNorms[i * 9 + j * 3 + 1] = NORMALS[ni][1];
      meshNorms[i * 9 + j * 3 + 2] = NORMALS[ni][2];

      meshColors[i * 9 + j * 3] = pcColors[pi * 3];
      meshColors[i * 9 + j * 3 + 1] = pcColors[pi * 3 + 1];
      meshColors[i * 9 + j * 3 + 2] = pcColors[pi * 3 + 2];
    }}
  }}

  meshGeom.setAttribute('position', new THREE.BufferAttribute(meshPositions, 3));
  meshGeom.setAttribute('normal', new THREE.BufferAttribute(meshNorms, 3));
  meshGeom.setAttribute('color', new THREE.BufferAttribute(meshColors, 3));

  const meshMat = new THREE.MeshPhongMaterial({{
    vertexColors: true, side: THREE.DoubleSide,
    transparent: true, opacity: 0.7,
    shininess: 30, flatShading: false,
  }});
  meshObj = new THREE.Mesh(meshGeom, meshMat);
  scene.add(meshObj);

  // Wireframe
  const wireMat = new THREE.MeshBasicMaterial({{
    color: 0x4488ff, wireframe: true, transparent: true, opacity: 0.15
  }});
  wireframeObj = new THREE.Mesh(meshGeom.clone(), wireMat);
  wireframeObj.visible = false;
  scene.add(wireframeObj);
}}

// ─── Build Trajectory ───
let trajLine = null;
if (TRAJECTORY.length > 1) {{
  const trajGeom = new THREE.BufferGeometry();
  const trajPositions = new Float32Array(TRAJECTORY.length * 3);
  const trajColors = new Float32Array(TRAJECTORY.length * 3);
  for (let i = 0; i < TRAJECTORY.length; i++) {{
    trajPositions[i * 3] = TRAJECTORY[i][0];
    trajPositions[i * 3 + 1] = TRAJECTORY[i][1];
    trajPositions[i * 3 + 2] = TRAJECTORY[i][2];
    const t = i / TRAJECTORY.length;
    trajColors[i * 3] = 1.0 - t * 0.5;
    trajColors[i * 3 + 1] = 0.2 + t * 0.6;
    trajColors[i * 3 + 2] = 0.2;
  }}
  trajGeom.setAttribute('position', new THREE.BufferAttribute(trajPositions, 3));
  trajGeom.setAttribute('color', new THREE.BufferAttribute(trajColors, 3));
  const trajMat = new THREE.LineBasicMaterial({{ vertexColors: true, linewidth: 2 }});
  trajLine = new THREE.Line(trajGeom, trajMat);
  scene.add(trajLine);

  // Start / End markers
  const startGeo = new THREE.SphereGeometry(1.2, 12, 12);
  const startMat = new THREE.MeshBasicMaterial({{ color: 0x00ff88 }});
  const startMarker = new THREE.Mesh(startGeo, startMat);
  startMarker.position.set(TRAJECTORY[0][0], TRAJECTORY[0][1], TRAJECTORY[0][2]);
  trajLine.add(startMarker);

  const endGeo = new THREE.SphereGeometry(1.2, 12, 12);
  const endMat = new THREE.MeshBasicMaterial({{ color: 0xff4444 }});
  const endMarker = new THREE.Mesh(endGeo, endMat);
  const last = TRAJECTORY[TRAJECTORY.length - 1];
  endMarker.position.set(last[0], last[1], last[2]);
  trajLine.add(endMarker);
}}

// ─── Axes Helper ───
const axes = new THREE.AxesHelper(20);
scene.add(axes);

// ─── Grid ───
const grid = new THREE.GridHelper(100, 20, 0x333355, 0x222244);
scene.add(grid);

// ─── UI Bindings ───
const chkPoints = document.getElementById('chk-points');
const chkMesh = document.getElementById('chk-mesh');
const chkWireframe = document.getElementById('chk-wireframe');
const chkTrajectory = document.getElementById('chk-trajectory');
const chkAxes = document.getElementById('chk-axes');
const sliderPtSize = document.getElementById('slider-ptsize');
const sliderOpacity = document.getElementById('slider-opacity');
const selColorMode = document.getElementById('sel-colormode');

chkPoints.onchange = () => {{ pointCloud.visible = chkPoints.checked; }};
chkMesh.onchange = () => {{ if (meshObj) meshObj.visible = chkMesh.checked; }};
chkWireframe.onchange = () => {{ if (wireframeObj) wireframeObj.visible = chkWireframe.checked; }};
chkTrajectory.onchange = () => {{ if (trajLine) trajLine.visible = chkTrajectory.checked; }};
chkAxes.onchange = () => {{ axes.visible = chkAxes.checked; grid.visible = chkAxes.checked; }};

sliderPtSize.oninput = () => {{
  pcMat.size = parseFloat(sliderPtSize.value);
  document.getElementById('val-ptsize').textContent = sliderPtSize.value;
}};

sliderOpacity.oninput = () => {{
  if (meshObj) meshObj.material.opacity = parseFloat(sliderOpacity.value);
  document.getElementById('val-opacity').textContent = sliderOpacity.value;
}};

selColorMode.onchange = () => {{
  const mode = selColorMode.value;
  const newColors = computeColors(pointsFlat, mode);
  pcGeom.setAttribute('color', new THREE.BufferAttribute(newColors, 3));
  // Update mesh colors too
  if (meshObj) {{
    const mPos = meshObj.geometry.attributes.position.array;
    const mCol = meshObj.geometry.attributes.color.array;
    // Approximate: recompute from face vertex positions
    for (let i = 0; i < FACES.length; i++) {{
      for (let j = 0; j < 3; j++) {{
        const vi = FACES[i][j];
        const pi = vi < POINTS.length ? vi : 0;
        mCol[i * 9 + j * 3] = newColors[pi * 3];
        mCol[i * 9 + j * 3 + 1] = newColors[pi * 3 + 1];
        mCol[i * 9 + j * 3 + 2] = newColors[pi * 3 + 2];
      }}
    }}
    meshObj.geometry.attributes.color.needsUpdate = true;
  }}
}};

document.getElementById('btn-reset').onclick = () => controls.setPosition(80, 60, 80);
document.getElementById('btn-top').onclick = () => controls.setPosition(0, 120, 0.01);
document.getElementById('btn-front').onclick = () => controls.setPosition(0, 10, 120);
document.getElementById('btn-screenshot').onclick = () => {{
  renderer.render(scene, camera);
  const link = document.createElement('a');
  link.download = 'slam_3d_view.png';
  link.href = renderer.domElement.toDataURL('image/png');
  link.click();
}};

// ─── Render Loop ───
function animate() {{
  requestAnimationFrame(animate);
  renderer.render(scene, camera);
}}
animate();

// ─── Resize Handling ───
window.addEventListener('resize', () => {{
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}});
</script>
</body>
</html>"""
    return html


# ─────────────────────── Main ────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="3D Reconstruction & Viewer from SLAM point cloud")
    parser.add_argument("ply_file", help="Input .ply point cloud from visual SLAM")
    parser.add_argument("trajectory", nargs='?', default=None, help="Optional trajectory .csv (x,y,z per row)")
    parser.add_argument("--output", "-o", default="slam_viewer.html", help="Output HTML viewer file")
    parser.add_argument("--no-mesh", action="store_true", help="Skip mesh reconstruction, show points only")
    parser.add_argument("--alpha", type=float, default=None, help="Alpha parameter for surface reconstruction (auto if omitted)")
    parser.add_argument("--outlier-k", type=int, default=20, help="Neighbors for outlier removal (default: 20)")
    parser.add_argument("--outlier-std", type=float, default=2.0, help="Std-dev ratio for outlier removal (default: 2.0)")
    args = parser.parse_args()

    print(f"Reading point cloud: {args.ply_file}")
    pts = read_ply(args.ply_file)
    print(f"  Loaded {len(pts)} points")
    print(f"  Bounds: {pts.min(axis=0)} → {pts.max(axis=0)}")

    # Trajectory
    trajectory = None
    if args.trajectory:
        trajectory = read_trajectory(args.trajectory)
        if trajectory is not None:
            print(f"  Loaded trajectory: {len(trajectory)} poses")

    # Outlier removal
    print(f"Statistical outlier removal (k={args.outlier_k}, std={args.outlier_std})...")
    pts_clean, mask = statistical_outlier_removal(pts, k=args.outlier_k, std_ratio=args.outlier_std)
    n_removed = len(pts) - len(pts_clean)
    print(f"  Removed {n_removed} outliers → {len(pts_clean)} points remain")

    # Normal estimation
    print("Estimating normals...")
    normals = estimate_normals(pts_clean, k=15)
    print("  Done")

    # Surface reconstruction
    faces = np.empty((0, 3), dtype=int)
    if not args.no_mesh:
        print("Surface reconstruction...")
        faces = reconstruct_surface(pts_clean, normals, alpha=args.alpha)
        print(f"  Final mesh: {len(faces)} triangular faces")
    else:
        print("Mesh reconstruction skipped (--no-mesh)")

    # Generate HTML viewer
    print(f"Generating interactive viewer → {args.output}")
    html = generate_html_viewer(pts_clean, normals, faces, trajectory, title="SLAM 3D Reconstruction")

    with open(args.output, 'w') as f:
        f.write(html)

    # File size
    import os
    size_mb = os.path.getsize(args.output) / (1024 * 1024)
    print(f"  Output: {args.output} ({size_mb:.1f} MB)")
    print(f"\nOpen {args.output} in a browser to explore the 3D reconstruction!")
    print("Controls: left-drag=orbit, right-drag=pan, scroll=zoom, double-click=center")


if __name__ == "__main__":
    main()