#include "LidarController.h"
#include "CamController.h"
#include "crow_all.h"
#include <iostream>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <csignal>

// ===== CONFIGURATION =====
const int    TARGET_FPS      = 30;
const int    WEB_PORT        = 8080;   // Dashboard
const int    STREAM_PORT     = 8081;   // MJPEG stream (CamController)
const bool   FLIP_CAMERA     = true;
const bool   FLIP_LIDAR      = false;  // set true when mounted upside-down
const double MIN_AREA        = 200.0;
const double DEAD_ON_THRESH  = 5.0;
const int    CAMWIDTH        = 4096;
const int    CAMHEIGHT       = 800;
// =========================

// ---------- Shared state ----------
std::vector<LidarPoint> global_scan;
std::mutex              scan_mutex;

struct TrackingState {
    double      angle       = 0.0;
    double      distance    = 0.0;
    int         objectCount = 0;
    std::string command     = "NO RED DETECTED";
    int         actual_fps  = 0;
};
TrackingState global_tracking;
std::mutex    tracking_mutex;

std::atomic<bool> running{true};

// ---------- Signal handler ----------
void signalHandler(int) {
    std::cout << "\nShutting down..." << std::endl;
    running = false;
}

// ---------- Lidar thread ----------
void lidarThread(LidarController& lidar) {
    while (running) {
        auto scan = lidar.getLatestScan();
        if (!scan.empty()) {
            std::lock_guard<std::mutex> lock(scan_mutex);
            global_scan = scan;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

// ---------- Camera thread ----------
void cameraThread(CamController& cam) {
    const long frame_us = 1000000 / TARGET_FPS;
    int  frameCount = 0;
    auto fpsTimer   = std::chrono::steady_clock::now();

    while (running) {
        auto t0 = std::chrono::steady_clock::now();

        if (!cam.captureFrame()) {
            std::cerr << "Camera: failed to capture frame" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        frameCount++;

        // Update actual FPS once per second
        auto now  = std::chrono::steady_clock::now();
        double secs = std::chrono::duration<double>(now - fpsTimer).count();
        if (secs >= 1.0) {
            int fps = static_cast<int>(std::round(frameCount / secs));
            frameCount = 0;
            fpsTimer   = now;
            std::lock_guard<std::mutex> lock(tracking_mutex);
            global_tracking.actual_fps = fps;
        }

        CamController::Direction dir = cam.getDirection();
        {
            std::lock_guard<std::mutex> lock(tracking_mutex);
            global_tracking.angle       = dir.angle;
            global_tracking.distance    = dir.distance;
            global_tracking.objectCount = dir.objectCount;
            global_tracking.command     = dir.command;
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t0).count();
        long sleep = frame_us - elapsed;
        if (sleep > 0) usleep(sleep);
    }
}

// ---------- Dashboard HTML ----------
// The camera img src uses window.location.hostname so the port swap is automatic.
static const std::string DASHBOARD_HTML = R"html(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>SENTINEL</title>
<link href="https://fonts.googleapis.com/css2?family=Share+Tech+Mono&family=Rajdhani:wght@500;700&display=swap" rel="stylesheet">
<style>
  :root {
    --bg:      #080c10;
    --surface: #0d1520;
    --border:  #1a2d45;
    --accent:  #00e5ff;
    --warn:    #ff3d5a;
    --ok:      #39ff84;
    --dim:     #4a6080;
    --mono:    'Share Tech Mono', monospace;
  }
  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

  body {
    background: var(--bg);
    color: #c8ddef;
    font-family: 'Rajdhani', sans-serif;
    height: 100vh;
    display: flex;
    flex-direction: column;
    overflow: hidden;
  }

  /* Scanlines */
  body::after {
    content: '';
    position: fixed; inset: 0;
    background: repeating-linear-gradient(
      0deg, transparent, transparent 2px,
      rgba(0,0,0,.12) 2px, rgba(0,0,0,.12) 4px
    );
    pointer-events: none;
    z-index: 999;
  }

  /* ── Header ── */
  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 8px 20px;
    background: var(--surface);
    border-bottom: 1px solid var(--border);
    flex-shrink: 0;
  }
  header h1 {
    font-family: var(--mono);
    font-size: 1.1rem;
    color: var(--accent);
    text-shadow: 0 0 14px var(--accent);
    letter-spacing: .15em;
  }
  #pill {
    font-family: var(--mono);
    font-size: .65rem;
    padding: 3px 11px;
    border: 1px solid var(--ok);
    color: var(--ok);
    letter-spacing: .1em;
    animation: blink 2s ease-in-out infinite;
  }
  @keyframes blink { 0%,100%{opacity:1} 50%{opacity:.3} }

  /* ── Top row: lidar + camera ── */
  #top-row {
    display: flex;
    flex-direction: row;
    border-bottom: 1px solid var(--border);
    flex-shrink: 0;
  }

  /* Lidar */
  #lidar-panel {
    background: var(--surface);
    border-right: 1px solid var(--border);
    display: flex;
    flex-direction: column;
    align-items: center;
    padding: 12px 10px 8px;
    gap: 6px;
    flex-shrink: 0;
    width: 310px;
  }
  #lidar-panel .lbl {
    font-family: var(--mono);
    font-size: .6rem;
    color: var(--dim);
    letter-spacing: .12em;
    align-self: flex-start;
  }
  canvas#radar {
    width: 270px;
    height: 270px;
    border-radius: 50%;
    box-shadow: 0 0 24px rgba(0,229,255,.06);
  }
  #pt-count {
    font-family: var(--mono);
    font-size: .65rem;
    color: var(--dim);
  }

  /* Camera */
  #cam-panel {
    flex: 1;
    background: #000;
    display: flex;
    flex-direction: column;
    position: relative;
    min-width: 0;
  }
  #cam-panel .lbl {
    position: absolute;
    top: 7px; left: 10px;
    font-family: var(--mono);
    font-size: .6rem;
    color: var(--dim);
    letter-spacing: .12em;
    z-index: 2;
  }
  #cam-feed {
    width: 100%;
    height: 100%;
    object-fit: contain;
    display: block;
  }
  #cam-offline {
    display: none;
    position: absolute;
    inset: 0;
    align-items: center;
    justify-content: center;
    font-family: var(--mono);
    font-size: .75rem;
    color: var(--dim);
    letter-spacing: .1em;
    flex-direction: column;
    gap: 8px;
  }
  #cam-offline.show { display: flex; }
  .offline-dot {
    width: 8px; height: 8px;
    border-radius: 50%;
    background: var(--dim);
    animation: blink 1.4s ease-in-out infinite;
  }

  /* ── Bottom: stats ── */
  #stats-panel {
    flex: 1;
    background: var(--bg);
    padding: 16px 22px;
    display: flex;
    flex-direction: column;
    gap: 12px;
    overflow-y: auto;
    min-height: 0;
  }
  #stats-panel .lbl {
    font-family: var(--mono);
    font-size: .6rem;
    color: var(--dim);
    letter-spacing: .12em;
  }
  .sgrid {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 10px;
  }
  .sbox {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 3px;
    padding: 10px 14px;
  }
  .slbl {
    font-family: var(--mono);
    font-size: .56rem;
    color: var(--dim);
    letter-spacing: .1em;
    text-transform: uppercase;
    margin-bottom: 4px;
  }
  .sval {
    font-family: var(--mono);
    font-size: 1.45rem;
    color: var(--accent);
    text-shadow: 0 0 8px rgba(0,229,255,.35);
    transition: color .2s, text-shadow .2s;
  }
  .sval.ok   { color: var(--ok);   text-shadow: 0 0 8px rgba(57,255,132,.35); }
  .sval.warn { color: var(--warn); text-shadow: 0 0 8px rgba(255,61,90,.35); }
  .sval.dim  { color: var(--dim);  text-shadow: none; }

  /* Compass + command row */
  .bottom-row {
    display: grid;
    grid-template-columns: 1fr auto;
    gap: 10px;
    align-items: stretch;
  }
  .compass {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 3px;
    padding: 10px 14px;
    display: flex;
    flex-direction: column;
    gap: 7px;
  }
  .ctrack {
    width: 100%;
    height: 5px;
    background: var(--border);
    border-radius: 3px;
    position: relative;
  }
  #needle {
    position: absolute;
    width: 11px; height: 11px;
    background: var(--accent);
    border-radius: 50%;
    top: -3px;
    left: 50%;
    transform: translateX(-50%);
    box-shadow: 0 0 8px var(--accent);
    transition: left .12s ease;
  }
  .clabels {
    display: flex;
    justify-content: space-between;
    font-family: var(--mono);
    font-size: .5rem;
    color: var(--dim);
  }
  .cmdbox {
    background: var(--surface);
    border: 1px solid var(--border);
    border-left: 3px solid var(--accent);
    border-radius: 3px;
    padding: 10px 18px;
    font-family: var(--mono);
    font-size: 1rem;
    letter-spacing: .07em;
    color: var(--ok);
    white-space: nowrap;
    display: flex;
    align-items: center;
    transition: color .2s, border-color .2s;
  }
</style>
</head>
<body>

<header>
  <h1>// SENTINEL</h1>
  <span id="pill">● LIVE</span>
</header>

<!-- Top row: lidar left, camera right -->
<div id="top-row">

  <div id="lidar-panel">
    <span class="lbl">LIDAR — 360° SCAN</span>
    <canvas id="radar" width="270" height="270"></canvas>
    <span id="pt-count">— pts</span>
  </div>

  <div id="cam-panel">
    <span class="lbl">CAMERA — TARGET TRACKING</span>
    <img id="cam-feed" alt="">
    <div id="cam-offline" class="show">
      <div class="offline-dot"></div>
      <span>STREAM CONNECTING...</span>
    </div>
  </div>

</div>

<!-- Stats below -->
<div id="stats-panel">
  <span class="lbl">TRACKING DATA</span>

  <div class="sgrid">
    <div class="sbox"><div class="slbl">Objects</div><div class="sval dim" id="s-obj">0</div></div>
    <div class="sbox"><div class="slbl">Bearing</div><div class="sval" id="s-ang">0.0°</div></div>
    <div class="sbox"><div class="slbl">Lateral Offset</div><div class="sval" id="s-dist">0.00</div></div>
    <div class="sbox"><div class="slbl">Lidar Points</div><div class="sval" id="s-pts">0</div></div>
    <div class="sbox"><div class="slbl">Actual FPS</div><div class="sval" id="s-fps">0</div></div>
  </div>

  <div class="bottom-row">
    <div class="compass">
      <div class="slbl">BEARING — LEFT ← · → RIGHT</div>
      <div class="ctrack"><div id="needle"></div></div>
      <div class="clabels"><span>−90°</span><span>0°</span><span>+90°</span></div>
    </div>
    <div class="cmdbox" id="s-cmd">NO RED DETECTED</div>
  </div>
</div>

<script>
  const TARGET_FPS = 30; // mirrors C++ config — green if ≥80% of target
  // Point camera stream at port 8081 on the same host, no hardcoded IP
  const streamHost = window.location.hostname;
  const feed = document.getElementById('cam-feed');
  const offline = document.getElementById('cam-offline');

  feed.onload = () => { offline.classList.remove('show'); };
  feed.onerror = () => {
    offline.classList.add('show');
    // Retry after 2s
    setTimeout(() => { feed.src = `http://${streamHost}:8081`; }, 2000);
  };
  feed.src = `http://${streamHost}:8081`;

  // ── Radar ──
  const radar = document.getElementById('radar');
  const ctx   = radar.getContext('2d');
  const W = radar.width, H = radar.height, CX = W/2, CY = H/2, R = CX - 4;

  // View state: zoom (metres visible = maxRange/zoom), pan offset in pixels
  let zoom      = 1.0;   // 1 = full 10m view, 2 = 5m view, etc.
  let panX      = 0;     // pixel offset from centre
  let panY      = 0;
  let dragging  = false;
  let dragStart = { x: 0, y: 0, px: 0, py: 0 };
  let lastPoints = [];

  const MAX_RANGE = 10.0; // metres — matches C++ filter

  // Convert metres → canvas pixels given current zoom/pan
  function mToPx(mx, my) {
    const scale = (R / MAX_RANGE) * zoom;
    return {
      x: CX + panX + mx * scale,
      y: CY + panY - my * scale
    };
  }

  function drawRadar(points) {
    lastPoints = points;
    ctx.clearRect(0, 0, W, H);

    const scale = (R / MAX_RANGE) * zoom;
    // How many metres does each ring represent at this zoom?
    const ringStep = niceRingStep(MAX_RANGE / zoom);
    const totalRings = Math.ceil((MAX_RANGE / zoom) / ringStep) + 1;

    // Background — clip to circle
    ctx.save();
    ctx.beginPath(); ctx.arc(CX, CY, R, 0, Math.PI * 2); ctx.clip();

    const bg = ctx.createRadialGradient(CX,CY,0,CX,CY,R);
    bg.addColorStop(0,'#0d1a10'); bg.addColorStop(1,'#080c10');
    ctx.fillStyle = bg; ctx.fillRect(0,0,W,H);

    // Range rings (relative to panned origin)
    const originPx = mToPx(0, 0);
    ctx.lineWidth = 0.8;
    for (let i = 1; i <= totalRings; i++) {
      const rMetres = i * ringStep;
      const rPx     = rMetres * scale;
      ctx.strokeStyle = '#1a2d45';
      ctx.beginPath();
      ctx.arc(originPx.x, originPx.y, rPx, 0, Math.PI * 2);
      ctx.stroke();
      // Label — place at top of ring, nudge right
      const lx = originPx.x + 3;
      const ly = originPx.y - rPx + 11;
      if (ly > 4 && ly < H - 4 && lx > 4 && lx < W - 4) {
        ctx.fillStyle = '#4a6080';
        ctx.font = '9px Share Tech Mono';
        ctx.fillText(rMetres < 1 ? (rMetres*100).toFixed(0)+'cm' : rMetres.toFixed(rMetres < 10 ? 1 : 0)+'m', lx, ly);
      }
    }

    // Crosshairs through panned origin
    ctx.strokeStyle = '#1a2d45'; ctx.lineWidth = 0.6;
    ctx.beginPath(); ctx.moveTo(originPx.x, 0); ctx.lineTo(originPx.x, H); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, originPx.y); ctx.lineTo(W, originPx.y); ctx.stroke();

    // Forward ray (straight up from origin)
    ctx.strokeStyle = '#00e5ff33'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(originPx.x, originPx.y); ctx.lineTo(originPx.x, originPx.y - R * zoom); ctx.stroke();

    // Points
    points.forEach(p => {
      const rad = (p.a - 90) * Math.PI / 180;
      const mx  =  Math.cos(rad) * p.r;
      const my  =  Math.sin(rad) * p.r;  // +y = forward
      const px  = mToPx(mx, my);

      const g = ctx.createRadialGradient(px.x,px.y,0,px.x,px.y,4);
      g.addColorStop(0,'#39ff84aa'); g.addColorStop(1,'transparent');
      ctx.fillStyle = g; ctx.fillRect(px.x-4,px.y-4,8,8);
      ctx.fillStyle = '#39ff84'; ctx.fillRect(px.x-1,px.y-1,2,2);
    });

    // Origin dot
    ctx.shadowBlur = 10; ctx.shadowColor = '#00e5ff';
    ctx.fillStyle = '#00e5ff';
    ctx.beginPath(); ctx.arc(originPx.x, originPx.y, 3, 0, Math.PI*2); ctx.fill();
    ctx.shadowBlur = 0;

    ctx.restore(); // end clip

    // HUD: zoom level (outside clip so always visible)
    ctx.fillStyle = '#4a6080';
    ctx.font = '9px Share Tech Mono';
    ctx.fillText(`×${zoom.toFixed(1)}  ${(MAX_RANGE/zoom).toFixed(1)}m view`, 8, H - 8);
  }

  // Pick a human-friendly ring interval for the current view range
  function niceRingStep(rangeMetres) {
    const steps = [0.25, 0.5, 1, 2, 2.5, 5, 10];
    const ideal = rangeMetres / 4;
    return steps.reduce((prev, s) => Math.abs(s - ideal) < Math.abs(prev - ideal) ? s : prev);
  }

  // ── Mouse / touch interaction ──
  radar.style.cursor = 'grab';

  radar.addEventListener('wheel', e => {
    e.preventDefault();
    const factor = e.deltaY < 0 ? 1.15 : 1/1.15;
    zoom = Math.max(0.5, Math.min(20, zoom * factor));
    drawRadar(lastPoints);
  }, { passive: false });

  radar.addEventListener('mousedown', e => {
    dragging  = true;
    dragStart = { x: e.clientX, y: e.clientY, px: panX, py: panY };
    radar.style.cursor = 'grabbing';
  });
  window.addEventListener('mousemove', e => {
    if (!dragging) return;
    panX = dragStart.px + (e.clientX - dragStart.x);
    panY = dragStart.py + (e.clientY - dragStart.y);
    drawRadar(lastPoints);
  });
  window.addEventListener('mouseup', () => {
    dragging = false;
    radar.style.cursor = 'grab';
  });

  // Double-click to reset view
  radar.addEventListener('dblclick', () => {
    zoom = 1.0; panX = 0; panY = 0;
    drawRadar(lastPoints);
  });

  // Touch support
  let lastTouchDist = null;
  radar.addEventListener('touchstart', e => {
    if (e.touches.length === 1) {
      dragging  = true;
      dragStart = { x: e.touches[0].clientX, y: e.touches[0].clientY, px: panX, py: panY };
    }
    if (e.touches.length === 2) {
      dragging = false;
      lastTouchDist = Math.hypot(
        e.touches[0].clientX - e.touches[1].clientX,
        e.touches[0].clientY - e.touches[1].clientY
      );
    }
  }, { passive: true });
  radar.addEventListener('touchmove', e => {
    e.preventDefault();
    if (e.touches.length === 1 && dragging) {
      panX = dragStart.px + (e.touches[0].clientX - dragStart.x);
      panY = dragStart.py + (e.touches[0].clientY - dragStart.y);
      drawRadar(lastPoints);
    }
    if (e.touches.length === 2 && lastTouchDist !== null) {
      const dist = Math.hypot(
        e.touches[0].clientX - e.touches[1].clientX,
        e.touches[0].clientY - e.touches[1].clientY
      );
      zoom = Math.max(0.5, Math.min(20, zoom * (dist / lastTouchDist)));
      lastTouchDist = dist;
      drawRadar(lastPoints);
    }
  }, { passive: false });
  radar.addEventListener('touchend', () => {
    dragging = false; lastTouchDist = null;
  });

  // ── Polling ──
  function sv(id, text, cls) {
    const el = document.getElementById(id);
    el.textContent = text;
    if (cls !== null) el.className = 'sval ' + (cls || '');
  }

  async function fetchLidar() {
    try {
      const d = await (await fetch('/data')).json();
      drawRadar(d);
      document.getElementById('pt-count').textContent = d.length + ' pts';
      sv('s-pts', d.length, null);
    } catch(e) {}
  }

  async function fetchTracking() {
    try {
      const d = await (await fetch('/tracking')).json();
      sv('s-obj',  d.objects,              d.objects > 0 ? 'ok' : 'dim');
      sv('s-ang',  d.angle.toFixed(1)+'°', Math.abs(d.angle) < 5 ? 'ok' : 'warn');
      sv('s-dist', d.distance.toFixed(2),  null);
      sv('s-fps',  d.actual_fps, d.actual_fps >= TARGET_FPS * 0.8 ? 'ok' : 'warn');

      const cmd = document.getElementById('s-cmd');
      cmd.textContent = d.command;
      const isF = d.command === 'FORWARD', isN = d.command.startsWith('NO');
      cmd.style.color           = isF ? 'var(--ok)' : isN ? 'var(--dim)' : 'var(--warn)';
      cmd.style.borderLeftColor = isF ? 'var(--ok)' : isN ? 'var(--dim)' : 'var(--warn)';

      const pct = Math.max(0, Math.min(100, ((d.angle + 90) / 180) * 100));
      document.getElementById('needle').style.left = pct + '%';
    } catch(e) {}
  }

  function loop() { fetchLidar(); fetchTracking(); setTimeout(loop, 100); }
  loop();
</script>
</body>
</html>
<!-- " -->
)html";

// ---------- main ----------
int main() {
    signal(SIGINT, signalHandler);

    // --- Lidar ---
    LidarController lidar("/dev/ttyUSB0", FLIP_LIDAR); // set true when mounted upside-down
    if (!lidar.initialize()) {
        std::cerr << "Failed to initialize lidar" << std::endl;
        return -1;
    }

    // --- Camera ---
    CamController cam(0, CAMWIDTH, CAMHEIGHT);

	//SET COLOUR HERE!
	cam.addTrackedColor("red", 0, 10, 100, 100, 170, 180, 100, 100);
	cam.addTrackedColor("purple", 130, 160, 100, 100, 130, 160, 100, 100);
	
    cam.setFlip(FLIP_CAMERA);
    cam.setTrackingStrategy(CamController::TrackingStrategy::LARGEST);
    cam.setMinArea(MIN_AREA);
    cam.setDeadOnThreshold(DEAD_ON_THRESH);

    if (!cam.initialize()) {
        std::cerr << "Failed to initialize camera" << std::endl;
        return -1;
    }

    // CamController's built-in MJPEG server on 8081
    cam.enableStreaming(true, STREAM_PORT);
    std::cout << "MJPEG stream : http://<your-pi-ip>:" << STREAM_PORT << std::endl;

    // --- Threads ---
    std::thread t_lidar(lidarThread, std::ref(lidar));
    std::thread t_cam(cameraThread, std::ref(cam));

    // --- Crow dashboard on 8080 ---
    crow::SimpleApp app;

    CROW_ROUTE(app, "/")([&](){ return DASHBOARD_HTML; });

    CROW_ROUTE(app, "/data")([&](){
        crow::json::wvalue x;
        std::lock_guard<std::mutex> lock(scan_mutex);
        for (size_t i = 0; i < global_scan.size(); i++) {
            float angle = global_scan[i].angle;
            x[i]["a"] = angle;
            x[i]["r"] = global_scan[i].range;
        }
        return x;
    });

    CROW_ROUTE(app, "/tracking")([&](){
        crow::json::wvalue x;
        std::lock_guard<std::mutex> lock(tracking_mutex);
        x["objects"]    = global_tracking.objectCount;
        x["angle"]      = global_tracking.angle;
        x["distance"]   = global_tracking.distance;
        x["command"]    = global_tracking.command;
        x["actual_fps"] = global_tracking.actual_fps;
        return x;
    });

    std::cout << "Dashboard    : http://<your-pi-ip>:" << WEB_PORT << std::endl;
    app.port(WEB_PORT).multithreaded().run();

    running = false;
    t_lidar.join();
    t_cam.join();
    cam.release();

    return 0;
}
