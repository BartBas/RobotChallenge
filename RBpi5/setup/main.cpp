//  setup/main.cpp
//  ──────────────────────────────────────────────────────────────────
//  Robot Setup Tool
//
//  Starts a tiny web server on port 8888.  Open in any browser:
//    http://<your-pi-ip>:8888
//
//  Page 1 – LIDAR     : polar radar, highlights points < 30 cm,
//                       shows their angles so you can paste exclusion
//                       zones straight into config.txt
//  Page 2 – COLOUR    : live camera feed analysed for the top
//                       red/purple-ish blobs; draws bounding boxes and
//                       shows the exact config.txt color = … line
//  Page 3 – COLLECTOR : live feed with draggable guide lines; place the
//                       cup under the collector arm, click to sample its
//                       X position, get ready-to-paste config lines.
//  ──────────────────────────────────────────────────────────────────

#include "../src/LidarController.h"
#include "../src/CamController.h"
#include "../src/Config.h"
#include "crow_all.h"

#include <atomic>
#include <thread>
#include <mutex>
#include <vector>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <iostream>

// ── shared state ────────────────────────────────────────────────────
std::atomic<bool> running{true};

std::vector<LidarPoint> g_scan;
std::mutex              g_scan_mx;

cv::Mat    g_frame;
std::mutex g_frame_mx;

// Raw (un-annotated) frame for the collector tab
cv::Mat    g_raw_frame;
std::mutex g_raw_frame_mx;

// Detected colour candidates (red/purple spectrum)
struct ColourBlob {
    std::string configLine;   // ready-to-paste config.txt line
    cv::Rect    box;
    cv::Scalar  bgr;          // representative colour for the box
    int         hueCenter;
    int         satCenter;
    int         valCenter;
    double      area;
};
std::vector<ColourBlob> g_blobs;
std::mutex              g_blobs_mx;

// Current collector window values (read from config, adjustable live)
std::atomic<float> g_collectXMin{0.55f};
std::atomic<float> g_collectXMax{0.75f};

// ── lidar thread ────────────────────────────────────────────────────
void lidarThread(LidarController& lidar) {
    while (running) {
        auto scan = lidar.getLatestScan();
        if (!scan.empty()) {
            std::lock_guard<std::mutex> lk(g_scan_mx);
            g_scan = scan;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

// ── colour analysis thread ───────────────────────────────────────────
void camThread(CamController& cam) {
    while (running) {
        if (!cam.captureFrame()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        cv::Mat frame = cam.getFrame();
        if (frame.empty()) continue;

        // Store raw frame for collector tab (before any annotation)
        {
            std::lock_guard<std::mutex> lk(g_raw_frame_mx);
            g_raw_frame = frame.clone();
        }

        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Hue bands to search: red-low, red-high, purple/magenta
        struct Band { int hMin, hMax, sMin, vMin; std::string hint; };
        std::vector<Band> bands = {
            {  0,  10, 80, 80, "red"    },
            {170, 180, 80, 80, "red"    },
            {130, 160, 80, 80, "purple" },
        };

        std::vector<ColourBlob> blobs;

        for (const auto& band : bands) {
            cv::Mat mask;
            cv::inRange(hsv,
                cv::Scalar(band.hMin, band.sMin, band.vMin),
                cv::Scalar(band.hMax, 255,       255),
                mask);

            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {5,5});
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel);
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            for (const auto& c : contours) {
                double area = cv::contourArea(c);
                if (area < 300) continue;

                cv::Rect box = cv::boundingRect(c);
                cv::Mat roi  = hsv(box);
                cv::Scalar mean = cv::mean(roi);
                int h = (int)mean[0], s = (int)mean[1], v = (int)mean[2];

                int hLow  = std::max(0,   h - 10);
                int hHigh = std::min(180, h + 10);
                int sMin  = std::max(0,   s - 40);
                int vMin  = std::max(0,   v - 40);

                std::ostringstream cfg;
                if (band.hint == "red") {
                    cfg << "color = " << band.hint
                        << "  0 10 " << sMin << " " << vMin
                        << "   170 180 " << sMin << " " << vMin;
                } else {
                    cfg << "color = " << band.hint
                        << "  " << hLow << " " << hHigh
                        << " "  << sMin << " " << vMin
                        << "   " << hLow << " " << hHigh
                        << " "  << sMin << " " << vMin;
                }

                cv::Mat tmp(1, 1, CV_8UC3, cv::Scalar(h, s, v));
                cv::cvtColor(tmp, tmp, cv::COLOR_HSV2BGR);
                cv::Vec3b bgr = tmp.at<cv::Vec3b>(0,0);

                ColourBlob blob;
                blob.configLine = cfg.str();
                blob.box        = box;
                blob.bgr        = cv::Scalar(bgr[0], bgr[1], bgr[2]);
                blob.hueCenter  = h;
                blob.satCenter  = s;
                blob.valCenter  = v;
                blob.area       = area;
                blobs.push_back(blob);
            }
        }

        std::sort(blobs.begin(), blobs.end(),
            [](const ColourBlob& a, const ColourBlob& b){ return a.area > b.area; });
        if (blobs.size() > 3) blobs.resize(3);

        cv::Mat annotated = frame.clone();
        for (size_t i = 0; i < blobs.size(); i++) {
            cv::rectangle(annotated, blobs[i].box, blobs[i].bgr, 3);
            std::string lbl = "#" + std::to_string(i+1) + " H:" +
                              std::to_string(blobs[i].hueCenter);
            cv::putText(annotated, lbl,
                {blobs[i].box.x, blobs[i].box.y - 8},
                cv::FONT_HERSHEY_SIMPLEX, 0.6, blobs[i].bgr, 2);
        }

        {
            std::lock_guard<std::mutex> lk(g_frame_mx);
            g_frame = annotated;
        }
        {
            std::lock_guard<std::mutex> lk(g_blobs_mx);
            g_blobs = blobs;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// ── HTML dashboard ───────────────────────────────────────────────────
static const char* HTML = R"html(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Robot Setup Tool</title>
<style>
  body { background:#111; color:#eee; font-family:monospace; margin:0; padding:16px; }
  h1   { color:#0af; margin:0 0 12px; }
  h2   { color:#0af; margin:16px 0 6px; }
  .tabs { display:flex; gap:8px; margin-bottom:16px; }
  .tab  { padding:8px 20px; background:#222; border:1px solid #444;
          cursor:pointer; border-radius:4px; }
  .tab.active { background:#0af; color:#000; }
  .panel { display:none; }
  .panel.active { display:block; }

  /* Lidar */
  #radarWrap { position:relative; width:500px; height:500px; }
  #radar     { border-radius:50%; background:#0a0a0a;
               border:1px solid #333; display:block; }
  #closeList { margin-top:12px; }
  .close-item{ padding:4px 8px; margin:3px 0; background:#1a1a1a;
               border-left:3px solid #f80; font-size:13px; }
  pre.cfgbox { background:#0d1a0d; border:1px solid #1f4; padding:10px;
               margin-top:8px; white-space:pre-wrap; color:#4f4; font-size:13px; }

  /* Camera / colour */
  #camFeed   { max-width:640px; border:1px solid #333; display:block; }
  .blob-card { background:#1a1a1a; border:1px solid #333; padding:10px;
               margin:6px 0; border-radius:4px; }
  .blob-card pre { margin:4px 0; color:#4f4; }
  .swatch    { display:inline-block; width:20px; height:20px;
               vertical-align:middle; border-radius:3px; margin-right:6px; }

  /* Collector tab */
  #collectWrap { position:relative; display:inline-block; }
  #collectFeed { display:block; border:1px solid #333; max-width:640px; }
  #collectOverlay {
    position:absolute; top:0; left:0;
    width:100%; height:100%;
    cursor:crosshair;
    pointer-events:all;
  }
  .guide-line {
    position:absolute; top:0; height:100%;
    width:3px; transform:translateX(-50%);
    background: rgba(160,0,255,0.85);
    pointer-events:none;
  }
  #guideMin { }
  #guideMax { }
  .guide-label {
    position:absolute; top:4px;
    background:rgba(0,0,0,0.6);
    color:#c080ff; font-size:11px; padding:2px 4px;
    pointer-events:none;
  }
  #sampleMarker {
    position:absolute; top:0; height:100%;
    width:2px; background:#0f0;
    transform:translateX(-50%);
    pointer-events:none;
    display:none;
  }
  #sampleLabel {
    position:absolute; top:30px;
    background:rgba(0,0,0,0.7);
    color:#0f0; font-size:12px; padding:2px 6px;
    pointer-events:none;
    display:none;
  }
  .collect-instructions {
    color:#888; font-size:13px; margin:8px 0 12px;
  }
  #collectResult {
    margin-top:14px;
  }
  .step {
    background:#1a1a2a; border:1px solid #336;
    border-radius:6px; padding:12px; margin:8px 0;
  }
  .step h3 { color:#adf; margin:0 0 6px; font-size:14px; }
  .step p  { color:#888; font-size:12px; margin:0 0 6px; }
  .snap-btn {
    background:#0af; color:#000; border:none;
    padding:8px 18px; border-radius:4px; cursor:pointer;
    font-family:monospace; font-size:14px; font-weight:bold;
  }
  .snap-btn:hover { background:#08d; }
  .snap-btn:disabled { background:#444; color:#888; cursor:default; }
  #sliderRow { display:flex; gap:24px; align-items:flex-end; margin:10px 0; }
  .slider-group label { display:block; font-size:12px; color:#888; margin-bottom:4px; }
  .slider-group input[type=range] { width:200px; }
  .slider-group span { font-size:13px; color:#0af; margin-left:8px; }
</style>
</head>
<body>
<h1>🤖 Robot Setup Tool</h1>

<div class="tabs">
  <div class="tab active" onclick="show('lidar',this)">📡 Lidar Deadzones</div>
  <div class="tab"        onclick="show('colour',this)">🎨 Colour Picker</div>
  <div class="tab"        onclick="show('collector',this)">🎯 Collector Align</div>
</div>

<!-- ── LIDAR PANEL ─────────────────────────────────── -->
<div id="lidar" class="panel active">
  <h2>Lidar — close points (&lt; 30 cm)</h2>
  <p style="color:#888;font-size:13px">
    Points within 30 cm are shown in <span style="color:#f80">orange</span>.
    Cluster angles are printed below — copy them as
    <code>lidar_exclude</code> lines into config.txt.
  </p>
  <canvas id="radar" width="500" height="500"></canvas>
  <div id="closeList"></div>
  <h2>Suggested config.txt lines</h2>
  <pre id="cfgLidar" class="cfgbox">(waiting for scan…)</pre>
</div>

<!-- ── COLOUR PANEL ────────────────────────────────── -->
<div id="colour" class="panel">
  <h2>Colour Picker — red / purple blobs</h2>
  <p style="color:#888;font-size:13px">
    Detected blobs ranked by size. Copy the line under each blob
    into config.txt as a <code>color =</code> entry.
  </p>
  <img id="camFeed" src="/cam_feed" alt="camera feed">
  <div id="blobList"></div>
</div>

<!-- ── COLLECTOR ALIGN PANEL ───────────────────────── -->
<div id="collector" class="panel">
  <h2>Collector Alignment</h2>
  <p class="collect-instructions">
    Place a cup exactly under your collector arm, then follow the steps below.<br>
    The two <span style="color:#c080ff">purple lines</span> show the current
    collection window from config.txt.<br>
    Click anywhere on the image to sample the cup's X position.
  </p>

  <!-- Step 1: adjust window with sliders -->
  <div class="step">
    <h3>Step 1 — drag the guide lines to bracket the collector arm</h3>
    <p>Use the sliders to move the purple lines until they frame where the cup
       needs to sit for a clean pickup. The values update live on the image.</p>
    <div id="sliderRow">
      <div class="slider-group">
        <label>Left guide (X min)</label>
        <input type="range" id="slMin" min="0" max="100" value="55"
               oninput="updateGuides()">
        <span id="slMinVal">55%</span>
      </div>
      <div class="slider-group">
        <label>Right guide (X max)</label>
        <input type="range" id="slMax" min="0" max="100" value="75"
               oninput="updateGuides()">
        <span id="slMaxVal">75%</span>
      </div>
    </div>
  </div>

  <!-- Live feed with overlay -->
  <div id="collectWrap">
    <img id="collectFeed" src="/cam_feed_raw" alt="collector feed" width="640">
    <div id="collectOverlay" onclick="sampleClick(event)">
      <!-- Left purple guide -->
      <div class="guide-line" id="guideMin">
        <div class="guide-label" id="guideMinLbl">xMin 55%</div>
      </div>
      <!-- Right purple guide -->
      <div class="guide-line" id="guideMax">
        <div class="guide-label" id="guideMaxLbl">xMax 75%</div>
      </div>
      <!-- Green sample marker (appears on click) -->
      <div id="sampleMarker"></div>
      <div id="sampleLabel"></div>
    </div>
  </div>

  <!-- Step 2: click to sample -->
  <div class="step" style="margin-top:14px">
    <h3>Step 2 — click the centre of the cup in the image above</h3>
    <p>A green line will appear at the sampled X. Then click "Set as window
       centre" to auto-calculate a ±10% window around that point.</p>
    <button class="snap-btn" id="autoBtn" disabled onclick="autoWindow()">
      Set as window centre
    </button>
  </div>

  <!-- Result -->
  <div id="collectResult" style="display:none">
    <h2>Suggested config.txt lines</h2>
    <pre id="cfgCollect" class="cfgbox"></pre>
  </div>
</div>

<script>
// ── tab switching ─────────────────────────────────────────────────
function show(id, el) {
  document.querySelectorAll('.panel').forEach(p => p.classList.remove('active'));
  document.querySelectorAll('.tab'  ).forEach(t => t.classList.remove('active'));
  document.getElementById(id).classList.add('active');
  el.classList.add('active');
}

// ── radar ─────────────────────────────────────────────────────────
const canvas = document.getElementById('radar');
const ctx    = canvas.getContext('2d');
const CX = 250, CY = 250, R = 240;

function drawRadar(points) {
  ctx.clearRect(0, 0, 500, 500);
  ctx.strokeStyle = '#1a3a1a';
  ctx.lineWidth = 1;
  [1, 2, 4, 8].forEach(d => {
    let r = (d / 8) * R;
    ctx.beginPath(); ctx.arc(CX, CY, r, 0, Math.PI*2); ctx.stroke();
    ctx.fillStyle = '#2a4a2a';
    ctx.fillText(d + 'm', CX + r + 2, CY - 2);
  });
  ctx.strokeStyle = '#1a3a1a';
  for (let a = 0; a < 360; a += 30) {
    let rad = (a - 90) * Math.PI / 180;
    ctx.beginPath();
    ctx.moveTo(CX, CY);
    ctx.lineTo(CX + Math.cos(rad)*R, CY + Math.sin(rad)*R);
    ctx.stroke();
    ctx.fillStyle = '#4a6a4a';
    ctx.fillText(a+'°', CX + Math.cos(rad)*(R+14) - 8,
                        CY + Math.sin(rad)*(R+14) + 4);
  }
  points.forEach(p => {
    let rad  = (p.a - 90) * Math.PI / 180;
    let dist = Math.min(p.r, 8.0);
    let px   = CX + Math.cos(rad) * (dist / 8) * R;
    let py   = CY + Math.sin(rad) * (dist / 8) * R;
    let close = p.r < 0.3;
    ctx.beginPath();
    ctx.arc(px, py, close ? 4 : 2, 0, Math.PI*2);
    ctx.fillStyle = close ? '#ff8800' : '#00cc44';
    ctx.fill();
  });
  ctx.beginPath(); ctx.arc(CX, CY, 6, 0, Math.PI*2);
  ctx.fillStyle = '#00aaff'; ctx.fill();
}

function clusterAngles(pts) {
  if (!pts.length) return [];
  let sorted = pts.map(p => p.a).sort((a,b) => a-b);
  let clusters = [], cur = [sorted[0]];
  for (let i = 1; i < sorted.length; i++) {
    if (sorted[i] - sorted[i-1] < 8) cur.push(sorted[i]);
    else { clusters.push(cur); cur = [sorted[i]]; }
  }
  clusters.push(cur);
  return clusters.map(c => ({
    min: Math.min(...c),
    max: Math.max(...c),
    count: c.length
  }));
}

async function pollLidar() {
  try {
    let r = await fetch('/lidar_data');
    let pts = await r.json();
    drawRadar(pts);
    let close = pts.filter(p => p.r < 0.3);
    let clusters = clusterAngles(close);
    let listEl = document.getElementById('closeList');
    listEl.innerHTML = close.length === 0
      ? '<p style="color:#888">No points within 30 cm</p>'
      : clusters.map(c =>
          `<div class="close-item">` +
          `📍 ${c.count} pts  |  angles ${c.min.toFixed(1)}° – ${c.max.toFixed(1)}°` +
          `</div>`).join('');
    let cfgEl = document.getElementById('cfgLidar');
    if (clusters.length === 0) {
      cfgEl.textContent = '# No close points detected';
    } else {
      cfgEl.textContent = clusters
        .map(c => `lidar_exclude = ${Math.max(0, Math.floor(c.min)-3)} ${Math.min(360, Math.ceil(c.max)+3)}`)
        .join('\n');
    }
  } catch(e) {}
  setTimeout(pollLidar, 200);
}

// ── colour blobs ──────────────────────────────────────────────────
async function pollBlobs() {
  try {
    let r = await fetch('/blob_data');
    let blobs = await r.json();
    let el = document.getElementById('blobList');
    if (!blobs.length) {
      el.innerHTML = '<p style="color:#888">No red/purple blobs detected</p>';
    } else {
      el.innerHTML = blobs.map((b, i) =>
        `<div class="blob-card">
          <span class="swatch" style="background:rgb(${b.r},${b.g},${b.b})"></span>
          <strong>#${i+1}</strong> — area ${b.area.toFixed(0)} px²
          &nbsp; H:${b.h} S:${b.s} V:${b.v}
          <pre>${b.cfg}</pre>
        </div>`
      ).join('');
    }
  } catch(e) {}
  setTimeout(pollBlobs, 500);
}

function refreshCam() {
  document.getElementById('camFeed').src = '/cam_feed?t=' + Date.now();
  setTimeout(refreshCam, 300);
}

// ── collector tab ─────────────────────────────────────────────────
let sampledNorm = null;   // 0..1 normalised X of clicked point

function updateGuides() {
  let minPct = parseInt(document.getElementById('slMin').value);
  let maxPct = parseInt(document.getElementById('slMax').value);

  // Clamp so min < max
  if (minPct >= maxPct) {
    maxPct = minPct + 1;
    document.getElementById('slMax').value = maxPct;
  }

  document.getElementById('slMinVal').textContent = minPct + '%';
  document.getElementById('slMaxVal').textContent = maxPct + '%';

  document.getElementById('guideMin').style.left = minPct + '%';
  document.getElementById('guideMax').style.left = maxPct + '%';
  document.getElementById('guideMinLbl').textContent = 'xMin ' + minPct + '%';
  document.getElementById('guideMaxLbl').textContent = 'xMax ' + maxPct + '%';

  rebuildConfig();
}

function sampleClick(evt) {
  let wrap = document.getElementById('collectWrap');
  let rect = wrap.getBoundingClientRect();
  let clickX = evt.clientX - rect.left;
  let wrapW  = rect.width;
  sampledNorm = Math.max(0, Math.min(1, clickX / wrapW));

  let pct = (sampledNorm * 100).toFixed(1);

  // Show green marker
  let marker = document.getElementById('sampleMarker');
  let label  = document.getElementById('sampleLabel');
  marker.style.left    = (sampledNorm * 100) + '%';
  marker.style.display = 'block';
  label.style.left     = (sampledNorm * 100 + 1) + '%';
  label.textContent    = '← cup centre ' + pct + '%';
  label.style.display  = 'block';

  document.getElementById('autoBtn').disabled = false;
  rebuildConfig();
}

function autoWindow() {
  if (sampledNorm === null) return;
  // Place a ±10% window around the sampled point
  let centre = sampledNorm * 100;
  let lo = Math.max(0,   Math.round(centre - 10));
  let hi = Math.min(100, Math.round(centre + 10));

  document.getElementById('slMin').value = lo;
  document.getElementById('slMax').value = hi;
  updateGuides();
}

function rebuildConfig() {
  let minPct = parseInt(document.getElementById('slMin').value);
  let maxPct = parseInt(document.getElementById('slMax').value);
  let minF   = (minPct / 100).toFixed(2);
  let maxF   = (maxPct / 100).toFixed(2);

  let lines = [];
  lines.push('# ── Collector window ─────────────────────────────');
  lines.push('brain_collect_x_min  = ' + minF);
  lines.push('brain_collect_x_max  = ' + maxF);

  if (sampledNorm !== null) {
    let pct = (sampledNorm * 100).toFixed(1);
    lines.push('');
    lines.push('# Cup centre was sampled at ' + pct + '% from the left edge');
  }

  let box = document.getElementById('cfgCollect');
  box.textContent = lines.join('\n');
  document.getElementById('collectResult').style.display = 'block';
}

function refreshCollectFeed() {
  document.getElementById('collectFeed').src = '/cam_feed_raw?t=' + Date.now();
  setTimeout(refreshCollectFeed, 300);
}

// Initialise guide line positions on load
updateGuides();

// Fetch current window values from server and pre-fill sliders
fetch('/collect_cfg').then(r => r.json()).then(d => {
  document.getElementById('slMin').value = Math.round(d.xmin * 100);
  document.getElementById('slMax').value = Math.round(d.xmax * 100);
  updateGuides();
}).catch(() => {});

pollLidar();
pollBlobs();
refreshCam();
refreshCollectFeed();
</script>
</body>
</html>
)html";

// ── main ─────────────────────────────────────────────────────────────
int main() {
    RobotConfig cfg;
    loadConfig("config.txt", cfg);

    // Seed shared atomics from config
    g_collectXMin.store(cfg.brainCollectXMin);
    g_collectXMax.store(cfg.brainCollectXMax);

    // ── Lidar ──
    LidarController lidar(cfg.lidarPort, cfg.flipLidar);
    if (!lidar.initialize()) {
        std::cerr << "[Setup] Lidar init failed on " << cfg.lidarPort << "\n";
        return 1;
    }

    // ── Camera ──
    CamController cam(0, 640, 480);
    cam.setFlip(cfg.flipCamera);
    if (!cam.initialize()) {
        std::cerr << "[Setup] Camera init failed\n";
        return 1;
    }

    // ── Background threads ──
    std::thread tLidar(lidarThread, std::ref(lidar));
    std::thread tCam  (camThread,   std::ref(cam));

    // ── Crow web server ──
    crow::SimpleApp app;

    CROW_ROUTE(app, "/")([](){ return std::string(HTML); });

    // Lidar scan data as JSON [{a, r}, ...]
    CROW_ROUTE(app, "/lidar_data")([&](){
        crow::json::wvalue arr;
        std::lock_guard<std::mutex> lk(g_scan_mx);
        for (size_t i = 0; i < g_scan.size(); i++) {
            arr[i]["a"] = g_scan[i].angle;
            arr[i]["r"] = g_scan[i].range;
        }
        return arr;
    });

    // Blob data as JSON
    CROW_ROUTE(app, "/blob_data")([&](){
        crow::json::wvalue arr;
        std::lock_guard<std::mutex> lk(g_blobs_mx);
        for (size_t i = 0; i < g_blobs.size(); i++) {
            arr[i]["cfg"]  = g_blobs[i].configLine;
            arr[i]["h"]    = g_blobs[i].hueCenter;
            arr[i]["s"]    = g_blobs[i].satCenter;
            arr[i]["v"]    = g_blobs[i].valCenter;
            arr[i]["area"] = g_blobs[i].area;
            arr[i]["r"] = (int)g_blobs[i].bgr[2];
            arr[i]["g"] = (int)g_blobs[i].bgr[1];
            arr[i]["b"] = (int)g_blobs[i].bgr[0];
        }
        return arr;
    });

    // Annotated camera feed (colour tab)
    CROW_ROUTE(app, "/cam_feed")([&](){
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lk(g_frame_mx);
            frame = g_frame.clone();
        }
        if (frame.empty()) return crow::response(503, "no frame yet");
        std::vector<uchar> buf;
        cv::imencode(".jpg", frame, buf, {cv::IMWRITE_JPEG_QUALITY, 80});
        crow::response res(200);
        res.set_header("Content-Type",  "image/jpeg");
        res.set_header("Cache-Control", "no-store");
        res.body.assign(buf.begin(), buf.end());
        return res;
    });

    // Raw camera feed (collector tab — no colour annotation)
    CROW_ROUTE(app, "/cam_feed_raw")([&](){
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lk(g_raw_frame_mx);
            frame = g_raw_frame.clone();
        }
        if (frame.empty()) return crow::response(503, "no frame yet");
        std::vector<uchar> buf;
        cv::imencode(".jpg", frame, buf, {cv::IMWRITE_JPEG_QUALITY, 80});
        crow::response res(200);
        res.set_header("Content-Type",  "image/jpeg");
        res.set_header("Cache-Control", "no-store");
        res.body.assign(buf.begin(), buf.end());
        return res;
    });

    // Current collector window values (so the page pre-fills from config)
    CROW_ROUTE(app, "/collect_cfg")([&](){
        crow::json::wvalue j;
        j["xmin"] = (double)g_collectXMin.load();
        j["xmax"] = (double)g_collectXMax.load();
        return j;
    });

    std::cout << "\n========================================\n"
              << "  Robot Setup Tool\n"
              << "  Open in your browser:\n"
              << "  http://<your-pi-ip>:8888\n"
              << "========================================\n\n";

    app.port(8888).multithreaded().run();

    running = false;
    tLidar.join();
    tCam.join();
    cam.release();
    return 0;
}
