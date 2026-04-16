/*
 * app.js — Pipeline Inspection Dashboard Client
 * ───────────────────────────────────────────────
 * Connects via WebSocket for real-time updates.
 * Fetches REST endpoints for initial data and on-demand refreshes.
 */

// ── Tab switching ──
document.querySelectorAll('.tab').forEach(btn => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
    document.querySelectorAll('.panel').forEach(p => p.classList.remove('active'));
    btn.classList.add('active');
    document.getElementById(btn.dataset.tab).classList.add('active');

    // Lazy-load tab data
    if (btn.dataset.tab === 'defects') fetchDefects();
    if (btn.dataset.tab === 'slam') fetchSLAM();
    if (btn.dataset.tab === 'system') fetchStatus();
  });
});

// ═══════════════════════════════════════════════════════════
//  WebSocket — real-time sensor + frame + detection updates
// ═══════════════════════════════════════════════════════════
let ws = null;
let wsRetry = 0;

function connectWS() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(`${proto}://${location.host}/ws/live`);

  ws.onopen = () => {
    wsRetry = 0;
    setWsStatus('ok', 'Connected');
  };

  ws.onclose = () => {
    setWsStatus('err', 'Disconnected');
    const delay = Math.min(2000 * Math.pow(2, wsRetry), 30000);
    wsRetry++;
    setTimeout(connectWS, delay);
  };

  ws.onerror = () => { ws.close(); };

  ws.onmessage = (evt) => {
    try {
      const data = JSON.parse(evt.data);
      handleWSMessage(data);
    } catch (e) {
      console.warn('WS parse error', e);
    }
  };
}

function setWsStatus(cls, label) {
  const dot = document.getElementById('ws-dot');
  dot.className = 'status-dot ' + cls;
  document.getElementById('ws-label').textContent = label;
}

function handleWSMessage(data) {
  // ── Video frame ──
  if (data.frame_b64) {
    const img = document.getElementById('feed-img');
    img.src = 'data:image/jpeg;base64,' + data.frame_b64;
    document.getElementById('feed-overlay').classList.add('hidden');
  }

  // ── Inference stats ──
  if (data.inference_ms !== undefined) {
    document.getElementById('inf-ms').textContent = data.inference_ms.toFixed(0);
  }

  // ── Detections ──
  if (data.detections) {
    document.getElementById('inf-count').textContent = data.detections.length;
    const ul = document.getElementById('det-list');
    if (data.detections.length === 0) {
      ul.innerHTML = '<li class="empty">None</li>';
    } else {
      ul.innerHTML = data.detections.map(d =>
        `<li><span class="det-class">${d.class_name}</span><span class="det-conf">${(d.confidence*100).toFixed(0)}%</span></li>`
      ).join('');
    }
  }

  // ── Sensors ──
  if (data.sensors) {
    updateSensors(data.sensors);
  }

  // ── Live SLAM frame ──
  if (data.slam_frame_b64) {
    const slamImg = document.getElementById('slam-feed-img');
    slamImg.src = 'data:image/jpeg;base64,' + data.slam_frame_b64;
    document.getElementById('slam-feed-overlay').classList.add('hidden');
  }
  if (data.slam_stats) {
    const st = data.slam_stats;
    document.getElementById('slam-live-pts').textContent = st.map_points || 0;
    document.getElementById('slam-live-traj').textContent = st.trajectory_length || 0;
    document.getElementById('slam-live-matches').textContent = st.matches || 0;
    document.getElementById('slam-live-inliers').textContent = st.inliers || 0;
  }

  // ── Playback state (control bar) ──
  if (data.playback) {
    updatePlaybackBar(data.playback);
  }
}

// ═══════════════════════════════════════════════════════════
//  Sensor display
// ═══════════════════════════════════════════════════════════
function updateSensors(s) {
  const v = (val, decimals=1) => val != null ? Number(val).toFixed(decimals) : '—';

  // Sensor grid
  document.getElementById('s-temp').textContent    = v(s.temperature_c);
  document.getElementById('s-hum').textContent     = v(s.humidity_pct, 0);
  document.getElementById('s-ultra').textContent   = v(s.ultrasonic_cm);
  document.getElementById('s-ir').textContent      = v(s.sharp_ir_cm);
  document.getElementById('s-avgdist').textContent = v(s.average_distance_cm);
  document.getElementById('s-speed').textContent   = v(s.speed_ms);
  document.getElementById('s-totdist').textContent = v(s.total_distance_m, 2);
  document.getElementById('s-uptime').textContent  = v(s.uptime_s, 1);
  document.getElementById('s-aq').textContent      = s.air_quality_label || '—';
  document.getElementById('s-aq-raw').textContent  = s.air_quality_raw != null ? `ADC: ${s.air_quality_raw}` : '';

  // Obstacle
  const obs = document.getElementById('s-obstacle');
  if (s.obstacle_detected === true) {
    obs.textContent = 'YES';
    obs.style.color = 'var(--red)';
  } else if (s.obstacle_detected === false) {
    obs.textContent = 'Clear';
    obs.style.color = 'var(--green)';
  } else {
    obs.textContent = '—';
    obs.style.color = '';
  }

  // Orientation
  if (s.roll_deg != null) {
    document.getElementById('s-orient').textContent =
      `${v(s.roll_deg)} / ${v(s.pitch_deg)} / ${v(s.yaw_deg)}`;
  }

  // Acceleration
  if (s.accel_x_ms2 != null) {
    document.getElementById('s-accel').textContent =
      `${v(s.accel_x_ms2, 2)} / ${v(s.accel_y_ms2, 2)} / ${v(s.accel_z_ms2, 2)}`;
  }

  // Tether
  const tet = document.getElementById('s-tether');
  if (s.tether_exceeded === true) {
    tet.textContent = 'EXCEEDED';
    tet.style.color = 'var(--red)';
  } else if (s.tether_exceeded === false) {
    tet.textContent = 'OK';
    tet.style.color = 'var(--green)';
  }

  // Quick sensors on Live Feed tab
  document.getElementById('qs-dist').textContent  = v(s.average_distance_cm);
  document.getElementById('qs-speed').textContent = v(s.speed_ms);
  document.getElementById('qs-air').textContent   = s.air_quality_label || '—';
}

// ═══════════════════════════════════════════════════════════
//  Defects table
// ═══════════════════════════════════════════════════════════
async function fetchDefects() {
  try {
    const res = await fetch('/api/defects?limit=200');
    const data = await res.json();
    renderDefects(data);
  } catch (e) {
    console.error('Fetch defects failed', e);
  }
}

function renderDefects(events) {
  const tbody = document.getElementById('defect-tbody');
  const stats = document.getElementById('defect-stats');

  if (!events || events.length === 0) {
    tbody.innerHTML = '<tr><td colspan="11" class="empty">No defects recorded yet.</td></tr>';
    stats.innerHTML = '';
    drawPipelineSchematic([]);
    return;
  }

  // Aggregate stats
  const counts = {};
  events.forEach(e => {
    counts[e.defect_class] = (counts[e.defect_class] || 0) + 1;
  });
  stats.innerHTML = Object.entries(counts)
    .map(([cls, n]) => `<span class="stat-badge">${cls}: ${n}</span>`)
    .join('');

  tbody.innerHTML = events.map(e => `
    <tr class="cls-${e.defect_class}">
      <td>${e.id || ''}</td>
      <td>${formatTime(e.timestamp)}</td>
      <td><strong>${e.defect_class}</strong></td>
      <td>${(e.confidence * 100).toFixed(1)}%</td>
      <td>${e.frame_number}</td>
      <td style="max-width:120px;overflow:hidden;text-overflow:ellipsis">${e.video_source || ''}</td>
      <td>${e.sensor_avg_distance_cm != null ? e.sensor_avg_distance_cm.toFixed(1) : '—'}</td>
      <td>${e.sensor_speed_ms != null ? e.sensor_speed_ms.toFixed(1) : '—'}</td>
      <td>${e.sensor_total_distance_m != null ? e.sensor_total_distance_m.toFixed(2) : '—'}</td>
      <td>${e.sensor_air_quality || '—'}</td>
      <td><button class="btn-del" data-id="${e.id}" title="Delete">✕</button></td>
    </tr>
  `).join('');

  // Attach delete handlers
  tbody.querySelectorAll('.btn-del').forEach(btn => {
    btn.addEventListener('click', () => deleteDefect(btn.dataset.id));
  });

  drawPipelineSchematic(events);
}

async function deleteDefect(id) {
  if (!id) return;
  try {
    const res = await fetch(`/api/defects/${id}`, { method: 'DELETE' });
    if (res.ok) {
      fetchDefects(); // reload table
    } else {
      console.error('Delete failed', await res.text());
    }
  } catch (e) {
    console.error('Delete request failed', e);
  }
}

// ═══════════════════════════════════════════════════════════
//  Pipeline Schematic — single continuous pipe, auto-scaling
// ═══════════════════════════════════════════════════════════

const DEFECT_COLORS = {
  'Deformation':  '#d29922',
  'Obstacle':     '#db6d28',
  'Rupture':      '#f85149',
  'Disconnect':   '#58a6ff',
  'Misalignment': '#bc8cff',
  'Deposition':   '#3fb950',
};

function drawPipelineSchematic(events) {
  // ── Determine pipe length from data ──
  // Use the furthest defect distance OR the robot's current odometer,
  // whichever is larger, with a small margin so the last marker isn't
  // pinned to the very edge.
  let maxDist = 0;
  events.forEach(e => {
    if (e.sensor_total_distance_m != null && e.sensor_total_distance_m > maxDist) {
      maxDist = e.sensor_total_distance_m;
    }
  });
  // Add 10% headroom, minimum 10 m so empty state looks reasonable
  const pipeLen = Math.max(maxDist * 1.1, 10);

  // ── Legend ──
  const legendDiv = document.getElementById('schematic-legend');
  const allClasses = Object.keys(DEFECT_COLORS);
  legendDiv.innerHTML = allClasses
    .map(c => `<span class="legend-item"><span class="legend-dot" style="background:${DEFECT_COLORS[c]}"></span>${c}</span>`)
    .join('');

  // ── Axis labels (4 evenly spaced ticks) ──
  const axisDiv = document.getElementById('schematic-axis');
  const ticks = [0, pipeLen * 0.25, pipeLen * 0.5, pipeLen * 0.75, pipeLen];
  axisDiv.innerHTML = ticks.map(t => `<span>${t.toFixed(1)} m</span>`).join('');

  // ── Canvas drawing ──
  const canvas = document.getElementById('pipeline-canvas');
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.parentElement.getBoundingClientRect();
  canvas.width  = rect.width * dpr;
  canvas.height = 48 * dpr;
  canvas.style.height = '48px';
  const ctx = canvas.getContext('2d');
  ctx.scale(dpr, dpr);
  const W = rect.width;
  const H = 48;

  ctx.clearRect(0, 0, W, H);

  const pipeY = H / 2;
  const pipeH = 18;
  const padX = 4;
  const usableW = W - padX * 2;
  const r = pipeH / 2;

  // ── Draw single continuous pipe ──
  ctx.fillStyle = '#1a4a6e';
  ctx.beginPath();
  ctx.moveTo(padX + r, pipeY - pipeH / 2);
  ctx.lineTo(padX + usableW - r, pipeY - pipeH / 2);
  ctx.arcTo(padX + usableW, pipeY - pipeH / 2, padX + usableW, pipeY, r);
  ctx.arcTo(padX + usableW, pipeY + pipeH / 2, padX + usableW - r, pipeY + pipeH / 2, r);
  ctx.lineTo(padX + r, pipeY + pipeH / 2);
  ctx.arcTo(padX, pipeY + pipeH / 2, padX, pipeY, r);
  ctx.arcTo(padX, pipeY - pipeH / 2, padX + r, pipeY - pipeH / 2, r);
  ctx.closePath();
  ctx.fill();

  // Subtle top highlight
  const grad = ctx.createLinearGradient(0, pipeY - pipeH / 2, 0, pipeY + pipeH / 2);
  grad.addColorStop(0, 'rgba(255,255,255,0.08)');
  grad.addColorStop(0.5, 'rgba(255,255,255,0)');
  grad.addColorStop(1, 'rgba(0,0,0,0.15)');
  ctx.fillStyle = grad;
  ctx.fill();

  // ── Plot defect markers ──
  const positioned   = events.filter(e => e.sensor_total_distance_m != null);
  const unpositioned = events.filter(e => e.sensor_total_distance_m == null);

  positioned.forEach(e => {
    const x = padX + (e.sensor_total_distance_m / pipeLen) * usableW;
    drawMarker(ctx, x, pipeY, 7, DEFECT_COLORS[e.defect_class] || '#ffffff');
  });

  // Fallback: spread unpositioned markers evenly
  unpositioned.forEach((e, i) => {
    const frac = (i + 1) / (unpositioned.length + 1);
    const x = padX + frac * usableW;
    drawMarker(ctx, x, pipeY, 7, DEFECT_COLORS[e.defect_class] || '#ffffff');
  });
}

function drawMarker(ctx, x, y, r, color) {
  ctx.beginPath();
  ctx.arc(x, y, r, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
  ctx.strokeStyle = '#0e1117';
  ctx.lineWidth = 1.5;
  ctx.stroke();
}

function formatTime(ts) {
  if (!ts) return '—';
  try {
    const d = new Date(ts);
    return d.toLocaleTimeString([], {hour:'2-digit',minute:'2-digit',second:'2-digit'})
         + ' ' + d.toLocaleDateString([], {month:'short',day:'numeric'});
  } catch { return ts; }
}

document.getElementById('btn-clear-defects').addEventListener('click', async () => {
  if (!confirm('Delete all defect records?')) return;
  try {
    await fetch('/api/defects', { method: 'DELETE' });
    fetchDefects();
  } catch (e) { console.error('Clear failed', e); }
});

document.getElementById('btn-refresh-defects').addEventListener('click', fetchDefects);

// Auto-refresh defects every 10s when on that tab
setInterval(() => {
  if (document.querySelector('.tab[data-tab="defects"]').classList.contains('active')) {
    fetchDefects();
  }
}, 10000);

// ═══════════════════════════════════════════════════════════
//  SLAM tab
// ═══════════════════════════════════════════════════════════
async function fetchSLAM() {
  try {
    const res = await fetch('/api/slam');
    const data = await res.json();

    // Status section
    document.getElementById('slam-msg').textContent  = data.message || '—';
    document.getElementById('slam-pct').textContent  = (data.progress_pct || 0).toFixed(0);
    document.getElementById('slam-bar').style.width  = (data.progress_pct || 0) + '%';
    document.getElementById('slam-last').textContent = data.last_run_time || 'Never';

    const ul = document.getElementById('slam-files');
    if (data.output_files && data.output_files.length > 0) {
      ul.innerHTML = data.output_files.map(f => `<li style="font-family:var(--font-mono);font-size:.78rem">${f}</li>`).join('');
        //ul.innerHTML = data.output_files.map(f => {
        //const name = f.split('/').pop();
        //return `<li style="font-family:var(--font-mono);font-size:.78rem">${name}</li>`;
      //}).join('');
    } else {
      ul.innerHTML = '<li class="empty">None yet</li>';
    }

    /*
    const link = document.getElementById('slam-viewer-link');
    const na   = document.getElementById('slam-viewer-na');
    if (data.viewer_html_path) {
      link.style.display = 'inline-block';
      na.style.display = 'none';
    } else {
      link.style.display = 'none';
      na.style.display = 'inline';
    }*/

    const wrap = document.getElementById('slam-viewer-wrap');
    const na   = document.getElementById('slam-viewer-na');
    const iframe = document.getElementById('slam-viewer-iframe');
    if (data.viewer_html_path) {
      wrap.style.display = '';
      na.style.display = 'none';
      // Only reload iframe if src isn't already set (avoid flicker)
      if (!iframe.src || !iframe.src.includes('/slam-viewer')) {
        iframe.src = '/slam-viewer';
      }
    } else {
      wrap.style.display = 'none';
      na.style.display = 'inline';
    }


    // Show/hide action cards based on context
    const videoCard = document.getElementById('slam-action-video');
    const liveCard  = document.getElementById('slam-action-live');

    // "Run on current video" — show when a video file is configured
    if (data.current_video_file) {
      videoCard.style.display = '';
      const fname = data.current_video_file.split('/').pop();
      document.getElementById('slam-current-file').textContent = fname;
    } else {
      videoCard.style.display = 'none';
    }

    // "Export live SLAM" — show live stats
    if (data.live_slam_stats && data.live_slam_stats.initialized) {
      liveCard.style.display = '';
      document.getElementById('slam-export-pts').textContent = data.live_slam_stats.map_points || 0;
      document.getElementById('slam-export-traj').textContent = data.live_slam_stats.trajectory_length || 0;
      // Enable/disable export button
      document.getElementById('btn-slam-export').disabled = !data.live_slam_has_data;
    } else {
      liveCard.style.display = '';
      document.getElementById('slam-export-pts').textContent = '0';
      document.getElementById('slam-export-traj').textContent = '0';
      document.getElementById('btn-slam-export').disabled = true;
    }

  } catch (e) {
    console.error('Fetch SLAM failed', e);
  }
}

// Manual run
document.getElementById('btn-slam-run').addEventListener('click', async () => {
  const input = document.getElementById('slam-video-input').value.trim();
  const msg = document.getElementById('slam-run-msg');
  if (!input) { msg.textContent = 'Enter a video path first.'; return; }
  try {
    const res = await fetch(`/api/slam/run?video_path=${encodeURIComponent(input)}`, {method:'POST'});
    const data = await res.json();
    msg.textContent = data.message || data.error || 'Done';
    setTimeout(fetchSLAM, 2000);
  } catch (e) {
    msg.textContent = 'Request failed: ' + e.message;
  }
});

// Run on current video
document.getElementById('btn-slam-run-current').addEventListener('click', async () => {
  const msg = document.getElementById('slam-run-current-msg');
  msg.textContent = 'Starting…';
  try {
    const res = await fetch('/api/slam/run-current', {method:'POST'});
    const data = await res.json();
    msg.textContent = data.message || data.error || '';
    setTimeout(fetchSLAM, 2000);
  } catch (e) {
    msg.textContent = 'Failed: ' + e.message;
  }
});

// Export live SLAM
document.getElementById('btn-slam-export').addEventListener('click', async () => {
  const msg = document.getElementById('slam-export-msg');
  msg.textContent = 'Exporting…';
  try {
    const res = await fetch('/api/slam/export-live', {method:'POST'});
    const data = await res.json();
    if (data.success) {
      msg.textContent = `Exported ${data.map_points} pts, ${data.trajectory_poses} poses`;
    } else {
      msg.textContent = data.error || 'Export failed';
    }
    setTimeout(fetchSLAM, 1000);
  } catch (e) {
    msg.textContent = 'Failed: ' + e.message;
  }
});

// Poll SLAM status when on that tab
setInterval(() => {
  if (document.querySelector('.tab[data-tab="slam"]').classList.contains('active')) {
    fetchSLAM();
  }
}, 3000);

// ═══════════════════════════════════════════════════════════
//  System Status tab
// ═══════════════════════════════════════════════════════════
async function fetchStatus() {
  try {
    const res = await fetch('/api/status');
    const d = await res.json();

    setIndicator('st-serial-dot', d.serial_connected);
    document.getElementById('st-serial-detail').textContent =
      d.serial_connected ? `Connected on ${d.serial_port}` : (d.serial_port === 'mock' ? 'Mock data mode' : 'Disconnected');

    setIndicator('st-camera-dot', d.camera_active);
    document.getElementById('st-camera-detail').textContent =
      d.camera_active ? d.video_source : 'Inactive';

    setIndicator('st-yolo-dot', d.yolo_loaded);
    document.getElementById('st-yolo-detail').textContent =
      d.yolo_loaded ? `Loaded (${d.inference_ms?.toFixed(0) || 0}ms/frame)` : 'Not loaded';

    setIndicator('st-slam-dot', d.slam_available);
    document.getElementById('st-slam-detail').textContent =
      d.slam_available ? 'Modules available' : 'Not available';

    setIndicator('st-db-dot', d.db_active);
    document.getElementById('st-db-detail').textContent =
      d.db_active ? `Active (${d.defect_count || 0} events)` : 'Inactive';

  } catch (e) {
    console.error('Fetch status failed', e);
  }
}

function setIndicator(id, ok) {
  const el = document.getElementById(id);
  el.className = 'status-indicator ' + (ok ? 'ok' : 'err');
}

// ═══════════════════════════════════════════════════════════
//  Video Control Bar
// ═══════════════════════════════════════════════════════════
let _isSeeking = false;  // true while user is dragging the slider
let _isPaused = false;

function fmtTime(seconds) {
  if (!seconds || seconds < 0) return '0:00';
  const m = Math.floor(seconds / 60);
  const s = Math.floor(seconds % 60);
  return m + ':' + String(s).padStart(2, '0');
}

function updatePlaybackBar(pb) {
  const bar = document.getElementById('video-controls');
  bar.style.display = 'flex';

  // Show/hide seek controls (only relevant for file sources)
  const seekOnly = [
    document.getElementById('vc-play'),
    document.getElementById('vc-current'),
    document.querySelector('.vc-seek-wrap'),
    document.getElementById('vc-duration'),
    document.querySelector('.vc-divider'),
  ];
  seekOnly.forEach(el => {
    if (el) el.style.display = pb.is_file ? '' : 'none';
  });

  if (pb.is_file) {
    _isPaused = pb.paused;
    document.getElementById('vc-play').textContent = pb.paused ? '▶' : '⏸';
    document.getElementById('vc-current').textContent = fmtTime(pb.current_s);
    document.getElementById('vc-duration').textContent = fmtTime(pb.duration_s);
    if (!_isSeeking) {
      document.getElementById('vc-seek').value = Math.round(pb.progress * 10000);
    }
  }

  // Update source button label
  const isFile = pb.source_type === 'file';
  document.getElementById('vc-source-icon').textContent = isFile ? '🎥' : '📷';
  document.getElementById('vc-source-label').textContent = isFile ? 'File' : 'Webcam';
}

// Play / Pause button
document.getElementById('vc-play').addEventListener('click', async () => {
  try {
    await fetch('/api/feed/toggle', { method: 'POST' });
  } catch (e) { console.error('Toggle failed', e); }
});

// Source toggle button
document.getElementById('vc-source').addEventListener('click', async () => {
  const current = document.getElementById('vc-source-label').textContent;
  const next = current === 'File' ? 'webcam' : 'file';
  try {
    await fetch(`/api/feed/source?source=${next}`, { method: 'POST' });
  } catch (e) { console.error('Source switch failed', e); }
});

// Open file picker
document.getElementById('vc-open-file').addEventListener('click', () => {
  document.getElementById('vc-file-input').click();
});

// Upload selected video file
document.getElementById('vc-file-input').addEventListener('change', async (e) => {
  const file = e.target.files[0];
  if (!file) return;
  const label = document.getElementById('vc-filename');
  label.textContent = 'Uploading…';
  try {
    const form = new FormData();
    form.append('file', file);
    const res = await fetch('/api/feed/upload-video', { method: 'POST', body: form });
    const data = await res.json();
    if (res.ok) {
      label.textContent = data.filename;
    } else {
      label.textContent = data.error || 'Upload failed';
    }
  } catch (err) {
    label.textContent = 'Upload error';
    console.error('Upload failed', err);
  }
  // Reset input so the same file can be re-selected
  e.target.value = '';
});

// Seek slider
const seekSlider = document.getElementById('vc-seek');
seekSlider.addEventListener('mousedown', () => { _isSeeking = true; });
seekSlider.addEventListener('touchstart', () => { _isSeeking = true; });

seekSlider.addEventListener('change', async () => {
  _isSeeking = false;
  const fraction = parseInt(seekSlider.value) / 10000;
  try {
    await fetch(`/api/feed/seek?fraction=${fraction}`, { method: 'POST' });
  } catch (e) { console.error('Seek failed', e); }
});
seekSlider.addEventListener('mouseup', () => { _isSeeking = false; });
seekSlider.addEventListener('touchend', () => { _isSeeking = false; });

// Keyboard shortcut: Space to toggle pause
document.addEventListener('keydown', (e) => {
  if (e.code === 'Space' && e.target === document.body) {
    e.preventDefault();
    document.getElementById('vc-play').click();
  }
});

// ═══════════════════════════════════════════════════════════
//  Init
// ═══════════════════════════════════════════════════════════
connectWS();
fetchStatus();
