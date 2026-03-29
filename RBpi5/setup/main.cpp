//  setup/main.cpp
//  ──────────────────────────────────────────────────────────────────
//  Robot Setup Tool
//
//  Starts a tiny web server on port 8888.  Open in any browser:
//    http://<your-pi-ip>:8888
//
//  Page 1 – LIDAR  : polar radar, highlights points < 30 cm,
//                    shows their angles so you can paste exclusion
//                    zones straight into config.txt
//  Page 2 – COLOUR : live camera feed analysed for the top
//                    red/purple-ish blobs; draws bounding boxes and
//                    shows the exact config.txt color = … line
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

cv::Mat   g_frame;
std::mutex g_frame_mx;

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
// Scans the full HSV hue range for red (0-10 + 170-180) and
// purple/magenta (130-170) blobs, picks the top 3 by area.
void camThread(CamController& cam) {
    while (running) {
        if (!cam.captureFrame()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        cv::Mat frame = cam.getFrame();
        if (frame.empty()) continue;

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

            // Morphological cleanup
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {5,5});
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel);
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            for (const auto& c : contours) {
                double area = cv::contourArea(c);
                if (area < 300) continue;

                cv::Rect box = cv::boundingRect(c);

                // Sample average HSV inside the box
                cv::Mat roi = hsv(box);
                cv::Scalar mean = cv::mean(roi);
                int h = (int)mean[0], s = (int)mean[1], v = (int)mean[2];

                // Build config line
                // For colours that don't wrap (purple) use same range for both bands
                int hLow  = std::max(0,   h - 10);
                int hHigh = std::min(180, h + 10);
                int sMin  = std::max(0,   s - 40);
                int vMin  = std::max(0,   v - 40);

                std::ostringstream cfg;
                if (band.hint == "red") {
                    // Red wraps around 0/180 — keep classic two-band format
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

                // Convert mean HSV → BGR for box colour
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

        // Keep top 3 by area (deduplicated roughly by box overlap)
        std::sort(blobs.begin(), blobs.end(),
            [](const ColourBlob& a, const ColourBlob& b){ return a.area > b.area; });
        if (blobs.size() > 3) blobs.resize(3);

        // Draw boxes onto frame copy
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

  /* Camera */
  #camFeed   { max-width:640px; border:1px solid #333; display:block; }
  .blob-card { background:#1a1a1a; border:1px solid #333; padding:10px;
               margin:6px 0; border-radius:4px; }
  .blob-card pre { margin:4px 0; color:#4f4; }
  .swatch    { display:inline-block; width:20px; height:20px;
               vertical-align:middle; border-radius:3px; margin-right:6px; }
</style>
</head>
<body>
<h1>🤖 Robot Setup Tool</h1>

<div class="tabs">
  <div class="tab active" onclick="show('lidar',this)">📡 Lidar Deadzones</div>
  <div class="tab"        onclick="show('colour',this)">🎨 Colour Picker</div>
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

  // Grid rings
  ctx.strokeStyle = '#1a3a1a';
  ctx.lineWidth = 1;
  [1, 2, 4, 8].forEach(d => {
    let r = (d / 8) * R;
    ctx.beginPath(); ctx.arc(CX, CY, r, 0, Math.PI*2); ctx.stroke();
    ctx.fillStyle = '#2a4a2a';
    ctx.fillText(d + 'm', CX + r + 2, CY - 2);
  });

  // Angle spokes every 30°
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

  // Points
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

  // Robot dot
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

// Reload cam image periodically
function refreshCam() {
  document.getElementById('camFeed').src = '/cam_feed?t=' + Date.now();
  setTimeout(refreshCam, 300);
}

pollLidar();
pollBlobs();
refreshCam();
</script>
</body>
</html>
)html";

// ── main ─────────────────────────────────────────────────────────────
int main() {
    // Load config for ports / flip flags
    RobotConfig cfg;
    loadConfig("config.txt", cfg);

    // ── Lidar ──
    LidarController lidar(cfg.lidarPort, cfg.flipLidar);
    // No exclusion zones in setup mode — we want to SEE everything
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
            // Convert BGR → RGB for the browser swatch
            arr[i]["r"] = (int)g_blobs[i].bgr[2];
            arr[i]["g"] = (int)g_blobs[i].bgr[1];
            arr[i]["b"] = (int)g_blobs[i].bgr[0];
        }
        return arr;
    });

    // MJPEG-ish: single JPEG snapshot (browser polls every 300 ms)
    CROW_ROUTE(app, "/cam_feed")([&](){
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lk(g_frame_mx);
            frame = g_frame.clone();
        }
        if (frame.empty()) {
            return crow::response(503, "no frame yet");
        }
        std::vector<uchar> buf;
        cv::imencode(".jpg", frame, buf,
                     {cv::IMWRITE_JPEG_QUALITY, 80});
        crow::response res(200);
        res.set_header("Content-Type",  "image/jpeg");
        res.set_header("Cache-Control", "no-store");
        res.body.assign(buf.begin(), buf.end());
        return res;
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
