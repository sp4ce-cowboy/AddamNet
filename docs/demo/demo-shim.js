// Demo mode for GitHub Pages: replays a recorded run of the dashboard.
// It stands in for the FastAPI server: the WebSocket at /ws/live and the /api/* routes.
(() => {
  const DATA = 'data/';
  const TICK_MS = 500;
  const ready = fetch(DATA + 'timeline.json').then(r => r.json());
  let T = null, tick = 0, paused = false, cleared = false;
  const deleted = new Set();
  let shift = 0;   // moves the recorded times to the visitor's clock
  ready.then(t => {
    T = t;
    const last = Math.max(...t.defects.map(d => Date.parse(d.timestamp)));
    shift = Date.now() - last;
  });
  const shifted = d => Object.assign({}, d, { timestamp: new Date(Date.parse(d.timestamp) + shift).toISOString() });

  const b64cache = new Map();
  function b64(path) {
    if (!b64cache.has(path)) {
      b64cache.set(path, fetch(path).then(r => r.blob()).then(blob => new Promise(res => {
        const fr = new FileReader();
        fr.onload = () => res(String(fr.result).split(',')[1]);
        fr.readAsDataURL(blob);
      })));
    }
    return b64cache.get(path);
  }
  const pad = n => String(n).padStart(3, '0');
  const detPath = i => `${DATA}det/${pad(i)}.jpg`;
  const slamPath = i => `${DATA}slam/${pad(i % T.slam_frames)}.jpg`;

  async function message(i) {
    const t = T.ticks[i];
    const [frame, slam] = await Promise.all([b64(detPath(i)), b64(slamPath(i))]);
    // warm the cache for the next tick
    const n = (i + 1) % T.ticks.length; b64(detPath(n)); b64(slamPath(n));
    return {
      sensors: t.sensors, frame_b64: frame, detections: t.detections, inference_ms: t.inference_ms,
      video_active: !paused, serial_connected: false,
      playback: Object.assign({}, t.playback, { paused }),
      slam_frame_b64: slam, slam_stats: t.slam_stats,
    };
  }

  function visibleDefects() {
    if (cleared) return [];
    const n = T.ticks[tick].defects;
    return T.defects.slice(0, n).filter(d => !deleted.has(String(d.id))).reverse().map(shifted);
  }

  const RealWebSocket = window.WebSocket;
  class DemoSocket {
    constructor(url) {
      if (!String(url).includes('/ws/live')) return new RealWebSocket(url);
      this.readyState = 0;
      ready.then(() => {
        this.readyState = 1;
        this.onopen && this.onopen({});
        this._run();
      });
    }
    async _run() {
      while (this.readyState === 1) {
        const started = Date.now();
        const msg = await message(tick);
        if (this.readyState !== 1) break;
        this.onmessage && this.onmessage({ data: JSON.stringify(msg) });
        if (!paused) tick = (tick + 1) % T.ticks.length;
        await new Promise(r => setTimeout(r, Math.max(0, TICK_MS - (Date.now() - started))));
      }
    }
    send() {}
    close() { this.readyState = 3; this.onclose && this.onclose({}); }
  }
  DemoSocket.CONNECTING = 0; DemoSocket.OPEN = 1; DemoSocket.CLOSING = 2; DemoSocket.CLOSED = 3;
  window.WebSocket = DemoSocket;

  const json = (body, status = 200) =>
    new Response(JSON.stringify(body), { status, headers: { 'Content-Type': 'application/json' } });
  const notInDemo = what => json({ success: false, error: `${what} isn't available in the online demo.`,
                                   message: `${what} isn't available in the online demo.` });

  const realFetch = window.fetch.bind(window);
  window.fetch = async (input, init = {}) => {
    const url = new URL(typeof input === 'string' ? input : input.url, location.href);
    const path = url.pathname.replace(/^.*?(\/api\/)/, '/api/');
    if (!path.startsWith('/api/')) return realFetch(input, init);
    await ready;
    const method = (init.method || 'GET').toUpperCase();
    const t = T.ticks[tick];

    if (path === '/api/defects' && method === 'GET') return json(visibleDefects());
    if (path === '/api/defects' && method === 'DELETE') { cleared = true; return json({ success: true }); }
    if (path.startsWith('/api/defects/') && method === 'DELETE') {
      deleted.add(path.split('/').pop()); return json({ success: true });
    }
    if (path === '/api/status') {
      return json(Object.assign({}, T.status, {
        camera_active: !paused, inference_ms: t.inference_ms, defect_count: visibleDefects().length,
      }));
    }
    if (path === '/api/slam') {
      return json(Object.assign({}, T.slam, { live_slam_has_data: true, live_slam_stats: t.slam_stats }));
    }
    if (path === '/api/sensors') return json(t.sensors);
    if (path === '/api/detections') return json(t.detections);
    if (path === '/api/feed/toggle') { paused = !paused; return json({ paused }); }
    if (path === '/api/feed/pause') { paused = true; return json({ paused }); }
    if (path === '/api/feed/resume') { paused = false; return json({ paused }); }
    if (path === '/api/feed/seek') {
      const f = parseFloat(url.searchParams.get('fraction') || '0');
      tick = Math.min(T.ticks.length - 1, Math.max(0, Math.round(f * (T.ticks.length - 1))));
      return json({ success: true });
    }
    if (path === '/api/feed/source') return notInDemo('Switching to a webcam');
    if (path === '/api/feed/upload-video') return notInDemo('Uploading a video');
    if (path.startsWith('/api/slam/run')) return notInDemo('Running SLAM on a new video');
    if (path === '/api/slam/export-live') {
      return json({ success: true, map_points: t.slam_stats.map_points, trajectory_poses: t.slam_stats.trajectory_length });
    }
    return json({ error: 'not found' }, 404);
  };
})();
