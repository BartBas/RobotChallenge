#include "LidarController.h"
#include "CamController.h"
#include "MotorController.h"
#include "crow_all.h"
#include <iostream>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <string>

// ===== CONFIGURATION =====
const int    TARGET_FPS      = 30;
const int    WEB_PORT        = 8080;
const int    STREAM_PORT     = 8081;
const bool   FLIP_CAMERA     = true;
const bool   FLIP_LIDAR      = true;
const double MIN_AREA        = 200.0;
const double DEAD_ON_THRESH  = 5.0;
const int    CAMWIDTH        = 4096;
const int    CAMHEIGHT       = 800;

// Brain tuning
const int    BRAIN_HZ            = 10;    // autonomous loop rate
const float  BRAIN_CLEAR_DIST    = 0.6f;  // metres — obstacle threshold (front arc)
const int    BRAIN_CHASE_SPEED   = 45;    // % speed when chasing target
const int    BRAIN_SEEK_SPEED    = 30;    // % speed when searching
const int    BRAIN_AVOID_SPEED   = 35;    // % speed when avoiding obstacle
const float  BRAIN_FRONT_ARC     = 40.0f; // ±degrees around heading counted as "front"
// =========================

std::atomic<bool> running{true};

// ══════════════════════════════════════════════════════════════════
//  Shared sensor state
// ══════════════════════════════════════════════════════════════════

std::vector<LidarPoint> global_scan;
std::mutex              scan_mutex;

struct TrackingState {
    double      angle       = 0.0;
    double      distance    = 0.0;
    int         objectCount = 0;
    std::string command     = "NO TARGET";
    int         actual_fps  = 0;
};
TrackingState global_tracking;
std::mutex    tracking_mutex;

// ══════════════════════════════════════════════════════════════════
//  MotorCommand — what any controller (manual or brain) wants
// ══════════════════════════════════════════════════════════════════

struct MotorCommand {
    bool   cmd_enable = true;
    int    direction  = 90;   // compass degrees, 0-359 (90 = forward)
    int    turn       = 0;    // 0=none 1=left 2=right
    int    speed      = 0;    // 0-100 %
    bool   pickup     = false;
};

// ══════════════════════════════════════════════════════════════════
//  MotorBus — single point of truth for the physical motor
//  Any thread calls drive() / eStop(); it serialises access.
// ══════════════════════════════════════════════════════════════════

struct MotorBus {
    MotorController*        hw      = nullptr;  // set after HW init
    mutable std::mutex      mtx;

    // Last commanded state (readable by dashboard)
    MotorCommand            last_cmd;
    bool                    estopped = false;

    // Which source is in control
    // "manual"  — Crow /motor route
    // "brain"   — autonomous brainThread
    std::string             source   = "manual";

    void drive(const MotorCommand& cmd, const std::string& src) {
        std::lock_guard<std::mutex> lk(mtx);
        last_cmd = cmd;
        source   = src;
        if (!hw || !hw->isOpen()) return;
        if (!cmd.cmd_enable) { hw->eStop(); return; }
        MotorController::TurnDirection td = MotorController::TurnDirection::NONE;
        if (cmd.turn == 1) td = MotorController::TurnDirection::LEFT;
        if (cmd.turn == 2) td = MotorController::TurnDirection::RIGHT;
        hw->drive(cmd.direction, td, cmd.speed, cmd.pickup);
    }

    void eStop(const std::string& src = "manual") {
        std::lock_guard<std::mutex> lk(mtx);
        estopped        = true;
        last_cmd.speed  = 0;
        source          = src;
        if (hw && hw->isOpen()) hw->eStop();
    }

    void resume() {
        std::lock_guard<std::mutex> lk(mtx);
        estopped = false;
    }

    MotorCommand getLastCmd() const {
        std::lock_guard<std::mutex> lk(mtx);
        return last_cmd;
    }
};

MotorBus global_motor_bus;

// ══════════════════════════════════════════════════════════════════
//  BrainState — what the autonomous brain is doing (for dashboard)
// ══════════════════════════════════════════════════════════════════

struct BrainState {
    bool        enabled     = false;
    std::string mode        = "IDLE";   // IDLE / SEEK / CHASE / AVOID
    std::string reason      = "";
    float       front_clear = 0.0f;     // closest obstacle in front arc (m)
};
BrainState  global_brain;
std::mutex  brain_mutex;

// ══════════════════════════════════════════════════════════════════
//  Signal handler
// ══════════════════════════════════════════════════════════════════

void signalHandler(int) {
    std::cout << "\nShutting down..." << std::endl;
    running = false;
}

// ══════════════════════════════════════════════════════════════════
//  Lidar thread
// ══════════════════════════════════════════════════════════════════

void lidarThread(LidarController& lidar) {
    while (running) {
        auto scan = lidar.getLatestScan();
        if (!scan.empty()) {
            std::lock_guard<std::mutex> lk(scan_mutex);
            global_scan = scan;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

// ══════════════════════════════════════════════════════════════════
//  Camera thread
// ══════════════════════════════════════════════════════════════════

void cameraThread(CamController& cam) {
    const long frame_us = 1000000 / TARGET_FPS;
    int  frameCount = 0;
    auto fpsTimer   = std::chrono::steady_clock::now();

    while (running) {
        auto t0 = std::chrono::steady_clock::now();

        if (!cam.captureFrame()) {
            std::cerr << "Camera: failed to capture frame\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
        frameCount++;

        auto now  = std::chrono::steady_clock::now();
        double secs = std::chrono::duration<double>(now - fpsTimer).count();
        if (secs >= 1.0) {
            int fps = static_cast<int>(std::round(frameCount / secs));
            frameCount = 0; fpsTimer = now;
            std::lock_guard<std::mutex> lk(tracking_mutex);
            global_tracking.actual_fps = fps;
        }

        CamController::Direction dir = cam.getDirection();
        {
            std::lock_guard<std::mutex> lk(tracking_mutex);
            global_tracking.angle       = dir.angle;
            global_tracking.distance    = dir.distance;
            global_tracking.objectCount = dir.objectCount;
            global_tracking.command     = dir.command;
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t0).count();
        if (long s = frame_us - elapsed; s > 0) usleep(s);
    }
}

// ══════════════════════════════════════════════════════════════════
//  Brain thread  — autonomous controller
//
//  Priority stack (highest wins):
//    1. E-stopped          → hold still
//    2. Obstacle too close → AVOID (turn away from obstacle)
//    3. Target visible     → CHASE (steer toward camera bearing)
//    4. No target          → SEEK  (slow spin to find target)
// ══════════════════════════════════════════════════════════════════

void brainThread() {
    const auto period = std::chrono::milliseconds(1000 / BRAIN_HZ);

    while (running) {
        auto t0 = std::chrono::steady_clock::now();

        // ── Check if brain is enabled ──
        {
            std::lock_guard<std::mutex> lk(brain_mutex);
            if (!global_brain.enabled) {
                std::this_thread::sleep_until(t0 + period);
                continue;
            }
        }

        // ── Read sensor snapshots ──
        std::vector<LidarPoint> scan;
        { std::lock_guard<std::mutex> lk(scan_mutex);    scan     = global_scan; }

        TrackingState tracking;
        { std::lock_guard<std::mutex> lk(tracking_mutex); tracking = global_tracking; }

        bool estopped;
        { std::lock_guard<std::mutex> lk(global_motor_bus.mtx); estopped = global_motor_bus.estopped; }

        // ── Obstacle detection — front arc ──
        float front_closest = 9999.f;
        float worst_angle   = 0.f;   // angle of nearest obstacle (for avoidance)
        for (auto& p : scan) {
            // Normalise angle relative to forward (90°), map to -180..+180
            float rel = p.angle - 90.f;
            while (rel >  180.f) rel -= 360.f;
            while (rel < -180.f) rel += 360.f;
            if (std::abs(rel) < BRAIN_FRONT_ARC && p.range < front_closest) {
                front_closest = p.range;
                worst_angle   = rel;
            }
        }

        // ── Decide mode + command ──
        MotorCommand cmd;
        std::string  mode, reason;

        if (estopped) {
            mode   = "IDLE";
            reason = "e-stop active";
            cmd.speed = 0; cmd.cmd_enable = false;

        } else if (front_closest < BRAIN_CLEAR_DIST) {
            // AVOID — turn away from obstacle
            mode   = "AVOID";
            reason = "obstacle at " + std::to_string((int)(front_closest * 100)) + "cm";
            cmd.speed     = BRAIN_AVOID_SPEED;
            cmd.direction = 90;                       // keep heading forward
            // Turn away from the side the obstacle is on
            cmd.turn = (worst_angle > 0) ? 2 : 1;    // 1=left, 2=right

        } else if (tracking.objectCount > 0) {
            // CHASE — steer toward target bearing
            mode   = "CHASE";
            reason = "target at " + std::to_string((int)tracking.angle) + "deg";
            cmd.speed = BRAIN_CHASE_SPEED;
            // Convert camera bearing (-90..+90) to drive direction (0..359)
            // Positive angle = target to the right → turn right
            int heading = 90 + static_cast<int>(tracking.angle);
            heading = ((heading % 360) + 360) % 360;
            cmd.direction = heading;
            cmd.turn      = 0;

        } else {
            // SEEK — slow rotation to find a target
            mode   = "SEEK";
            reason = "no target visible";
            cmd.speed     = BRAIN_SEEK_SPEED;
            cmd.direction = 90;
            cmd.turn      = 1; // spin left (arbitrary, could alternate)
        }

        // ── Publish command ──
        global_motor_bus.drive(cmd, "brain");

        {
            std::lock_guard<std::mutex> lk(brain_mutex);
            global_brain.mode        = mode;
            global_brain.reason      = reason;
            global_brain.front_clear = (front_closest > 99.f) ? -1.f : front_closest;
        }

        std::this_thread::sleep_until(t0 + period);
    }
}

// ---------- Dashboard HTML ----------
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
  body::after {
    content: '';
    position: fixed; inset: 0;
    background: repeating-linear-gradient(0deg, transparent, transparent 2px, rgba(0,0,0,.12) 2px, rgba(0,0,0,.12) 4px);
    pointer-events: none;
    z-index: 999;
  }

  /* ── Header ── */
  header {
    display: flex;
    align-items: center;
    justify-content: space-between;
    padding: 7px 16px;
    background: var(--surface);
    border-bottom: 1px solid var(--border);
    flex-shrink: 0;
    gap: 12px;
  }
  header h1 {
    font-family: var(--mono);
    font-size: 1rem;
    color: var(--accent);
    text-shadow: 0 0 14px var(--accent);
    letter-spacing: .15em;
    flex-shrink: 0;
  }
  @keyframes blink { 0%,100%{opacity:1} 50%{opacity:.3} }
  #pill {
    font-family: var(--mono);
    font-size: .6rem;
    padding: 3px 10px;
    border: 1px solid var(--ok);
    color: var(--ok);
    letter-spacing: .1em;
    animation: blink 2s ease-in-out infinite;
    flex-shrink: 0;
  }

  /* ── 3-state mode selector ── */
  .mode-group {
    display: flex;
    align-items: center;
    gap: 0;
    border: 1px solid var(--border);
    border-radius: 3px;
    overflow: hidden;
    flex-shrink: 0;
  }
  .mode-btn {
    font-family: var(--mono);
    font-size: .62rem;
    letter-spacing: .08em;
    padding: 5px 18px;
    background: var(--bg);
    border: none;
    border-right: 1px solid var(--border);
    color: var(--dim);
    cursor: pointer;
    transition: background .12s, color .12s;
    white-space: nowrap;
  }
  .mode-btn:last-child { border-right: none; }
  .mode-btn.active-off  { background: rgba(255,61,90,.18);  color: var(--warn); }
  .mode-btn.active-man  { background: rgba(0,229,255,.12);  color: var(--accent); }
  .mode-btn.active-auto { background: rgba(57,255,132,.12); color: var(--ok); }

  /* brain status strip next to mode */
  .brain-strip {
    font-family: var(--mono);
    font-size: .6rem;
    color: var(--dim);
    letter-spacing: .07em;
    display: flex;
    align-items: center;
    gap: 8px;
    flex-shrink: 0;
  }
  .bdot {
    width: 7px; height: 7px; border-radius: 50%;
    background: var(--dim);
    flex-shrink: 0;
    transition: background .2s, box-shadow .2s;
  }
  .bdot.seek  { background: var(--warn); box-shadow: 0 0 5px var(--warn); }
  .bdot.chase { background: var(--ok);   box-shadow: 0 0 5px var(--ok); }
  .bdot.avoid { background: var(--warn); box-shadow: 0 0 5px var(--warn); }
  #hdr-mode-lbl { transition: color .2s; }

  /* ── Top row ── */
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
    padding: 10px 10px 6px;
    gap: 4px;
    flex-shrink: 0;
    width: 300px;
  }
  .lbl {
    font-family: var(--mono);
    font-size: .56rem;
    color: var(--dim);
    letter-spacing: .12em;
    align-self: flex-start;
  }
  canvas#radar {
    width: 260px; height: 260px;
    border-radius: 50%;
  }
  #pt-count { font-family: var(--mono); font-size: .6rem; color: var(--dim); }

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
    position: absolute; top: 6px; left: 8px;
    z-index: 2;
  }
  #cam-feed { width: 100%; height: 100%; object-fit: contain; display: block; }
  #cam-offline {
    display: none; position: absolute; inset: 0;
    align-items: center; justify-content: center;
    font-family: var(--mono); font-size: .7rem;
    color: var(--dim); letter-spacing: .1em;
    flex-direction: column; gap: 8px;
  }
  #cam-offline.show { display: flex; }
  .offline-dot { width: 8px; height: 8px; border-radius: 50%; background: var(--dim); animation: blink 1.4s ease-in-out infinite; }

  /* ── D-pad panel ── */
  #dpad-panel {
    background: var(--surface);
    border-left: 1px solid var(--border);
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    padding: 12px 12px;
    gap: 10px;
    flex-shrink: 0;
    width: 210px;
  }
  .dpad {
    display: grid;
    grid-template-columns: repeat(3, 50px);
    grid-template-rows: repeat(3, 50px);
    gap: 4px;
  }
  .dp-btn {
    background: var(--surface);
    border: 1px solid var(--border);
    border-radius: 4px;
    color: var(--dim);
    font-size: 1.1rem;
    cursor: pointer;
    display: flex; align-items: center; justify-content: center;
    user-select: none;
    transition: background .08s, color .08s, border-color .08s;
    -webkit-tap-highlight-color: transparent;
    touch-action: none;
  }
  .dp-btn:active, .dp-btn.pressed { background: var(--accent); color: var(--bg); border-color: var(--accent); }
  .dp-btn.disabled { opacity: .3; pointer-events: none; }
  .dp-center {
    background: #0a1219; border: 1px solid var(--border); border-radius: 4px;
    display: flex; align-items: center; justify-content: center;
  }
  .dp-turn-row { display: flex; gap: 4px; width: 100%; }
  .dp-turn-btn {
    font-family: var(--mono); font-size: .58rem; letter-spacing: .03em;
    padding: 4px 0; flex: 1;
    background: var(--bg); border: 1px solid var(--border); border-radius: 3px;
    color: var(--dim); cursor: pointer; text-align: center;
    transition: color .1s, border-color .1s, background .1s;
    user-select: none;
  }
  .dp-turn-btn.active   { background: var(--accent); color: var(--bg); border-color: var(--accent); }
  .dp-turn-btn.disabled { opacity: .3; pointer-events: none; }

  /* ── Bottom row: stats + status ── */
  #bottom-row {
    display: flex;
    flex: 1;
    min-height: 0;
    overflow: hidden;
  }

  /* Tracking stats */
  #stats-panel {
    flex: 1;
    background: var(--bg);
    padding: 12px 18px;
    display: flex;
    flex-direction: column;
    gap: 10px;
    overflow-y: auto;
    min-height: 0;
    border-right: 1px solid var(--border);
  }
  .sgrid { display: grid; grid-template-columns: repeat(5, 1fr); gap: 8px; }
  .sbox  { background: var(--surface); border: 1px solid var(--border); border-radius: 3px; padding: 8px 12px; }
  .slbl  { font-family: var(--mono); font-size: .52rem; color: var(--dim); letter-spacing: .1em; text-transform: uppercase; margin-bottom: 3px; }
  .sval  { font-family: var(--mono); font-size: 1.3rem; color: var(--accent); text-shadow: 0 0 8px rgba(0,229,255,.35); transition: color .2s, text-shadow .2s; }
  .sval.ok   { color: var(--ok);   text-shadow: 0 0 8px rgba(57,255,132,.35); }
  .sval.warn { color: var(--warn); text-shadow: 0 0 8px rgba(255,61,90,.35); }
  .sval.dim  { color: var(--dim);  text-shadow: none; }
  .bottom-row-inner { display: grid; grid-template-columns: 1fr auto; gap: 8px; align-items: stretch; }
  .compass { background: var(--surface); border: 1px solid var(--border); border-radius: 3px; padding: 8px 12px; display: flex; flex-direction: column; gap: 6px; }
  .ctrack  { width: 100%; height: 5px; background: var(--border); border-radius: 3px; position: relative; }
  #needle  { position: absolute; width: 11px; height: 11px; background: var(--accent); border-radius: 50%; top: -3px; left: 50%; transform: translateX(-50%); box-shadow: 0 0 8px var(--accent); transition: left .12s ease; }
  .clabels { display: flex; justify-content: space-between; font-family: var(--mono); font-size: .5rem; color: var(--dim); }
  .cmdbox  { background: var(--surface); border: 1px solid var(--border); border-left: 3px solid var(--accent); border-radius: 3px; padding: 8px 14px; font-family: var(--mono); font-size: .9rem; letter-spacing: .07em; color: var(--ok); white-space: nowrap; display: flex; align-items: center; transition: color .2s, border-color .2s; }

  /* Status panel */
  #status-panel {
    background: var(--surface);
    width: 220px;
    flex-shrink: 0;
    padding: 12px 14px;
    display: flex;
    flex-direction: column;
    gap: 12px;
    overflow-y: auto;
  }
  .sp-title { font-family: var(--mono); font-size: .54rem; color: var(--dim); letter-spacing: .12em; text-transform: uppercase; margin-bottom: 4px; }
  .sp-row   { display: flex; justify-content: space-between; align-items: center; font-family: var(--mono); font-size: .62rem; color: var(--dim); margin-top: 4px; }
  .sp-val   { color: #c8ddef; }
  .sp-val.ok   { color: var(--ok); }
  .sp-val.warn { color: var(--warn); }

  /* brain mode badge */
  .mode-badge {
    font-family: var(--mono); font-size: .7rem; letter-spacing: .06em;
    padding: 4px 10px; border-radius: 2px; border: 1px solid var(--border);
    color: var(--dim); background: var(--bg);
    transition: color .2s, border-color .2s, background .2s;
    text-align: center;
  }
  .mode-badge.seek  { color: var(--warn);   border-color: var(--warn);   background: rgba(255,61,90,.08); }
  .mode-badge.chase { color: var(--ok);     border-color: var(--ok);     background: rgba(57,255,132,.08); }
  .mode-badge.avoid { color: var(--accent); border-color: var(--accent); background: rgba(0,229,255,.08); }

  /* pickup toggle */
  .mc-toggle { width: 38px; height: 20px; border-radius: 10px; background: var(--bg); border: 1px solid var(--border); position: relative; cursor: pointer; transition: background .15s, border-color .15s; flex-shrink: 0; }
  .mc-toggle.on { background: var(--accent); border-color: var(--accent); }
  .mc-toggle-thumb { width: 14px; height: 14px; border-radius: 50%; background: var(--dim); position: absolute; top: 2px; left: 2px; transition: left .15s, background .15s; pointer-events: none; }
  .mc-toggle.on .mc-toggle-thumb { left: 20px; background: var(--bg); }

  /* packet box */
  .packet-box { font-family: var(--mono); font-size: .58rem; color: var(--dim); letter-spacing: .03em; line-height: 1.8; }
  .packet-box span { color: var(--accent); }

  /* front-clear bar */
  .fc-bar-bg   { width: 100%; height: 5px; background: var(--border); border-radius: 3px; overflow: hidden; margin-top: 3px; }
  .fc-bar-fill { height: 100%; width: 100%; background: var(--ok); border-radius: 3px; transition: width .3s, background .3s; }
</style>
</head>
<body>

<header>
  <h1>// SENTINEL</h1>

  <!-- 3-state mode selector -->
  <div class="mode-group">
    <button class="mode-btn active-off" id="btn-off"  onclick="setMode('off')">&#9632; OFF</button>
    <button class="mode-btn"            id="btn-man"  onclick="setMode('manual')">&#9654; MANUAL</button>
    <button class="mode-btn"            id="btn-auto" onclick="setMode('auto')">&#9681; AUTO</button>
  </div>

  <!-- live brain status (shown when in auto mode) -->
  <div class="brain-strip">
    <div class="bdot" id="hdr-bdot"></div>
    <span id="hdr-mode-lbl">IDLE</span>
    <span id="hdr-reason" style="color:var(--dim);opacity:.7;max-width:180px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;"></span>
  </div>

  <span id="pill">&#9679; LIVE</span>
</header>

<!-- Single unified view -->
<div id="top-row">
  <div id="lidar-panel">
    <span class="lbl">LIDAR — 360° SCAN</span>
    <canvas id="radar" width="260" height="260"></canvas>
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

  <!-- D-pad -->
  <div id="dpad-panel">
    <span class="lbl">DRIVE CONTROL</span>

    <!-- Speed slider -->
    <div style="width:100%;">
      <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:4px;">
        <span class="lbl" style="align-self:auto;">SPEED</span>
        <span style="font-family:var(--mono);font-size:.75rem;color:var(--accent);" id="dp-spd">0</span><span style="font-family:var(--mono);font-size:.55rem;color:var(--dim);">%</span>
      </div>
      <input type="range" id="dp-slider" min="0" max="100" value="50" step="1" style="width:100%;accent-color:var(--accent);">
    </div>

    <!-- D-pad buttons -->
    <div class="dpad">
      <div></div>
      <button class="dp-btn" id="dp-fwd"   title="Forward [↑]">&#8593;</button>
      <div></div>
      <button class="dp-btn" id="dp-left"  title="Turn left [←]">&#8592;</button>
      <div class="dp-center"><div style="width:8px;height:8px;border-radius:50%;background:var(--border);"></div></div>
      <button class="dp-btn" id="dp-right" title="Turn right [→]">&#8594;</button>
      <div></div>
      <button class="dp-btn" id="dp-back"  title="Reverse [↓]">&#8595;</button>
      <div></div>
    </div>

    <div class="dp-turn-row">
      <button class="dp-turn-btn" id="dp-tleft"  title="Turn left [A]">[A] L</button>
      <button class="dp-turn-btn active" id="dp-tnone" title="Straight">STR</button>
      <button class="dp-turn-btn" id="dp-tright" title="Turn right [D]">R [D]</button>
    </div>
  </div>
</div>

<!-- Bottom row -->
<div id="bottom-row">

  <!-- Tracking stats -->
  <div id="stats-panel">
    <span class="lbl">TRACKING DATA</span>
    <div class="sgrid">
      <div class="sbox"><div class="slbl">Objects</div><div class="sval dim" id="s-obj">0</div></div>
      <div class="sbox"><div class="slbl">Bearing</div><div class="sval" id="s-ang">0.0°</div></div>
      <div class="sbox"><div class="slbl">Offset</div><div class="sval" id="s-dist">0.00</div></div>
      <div class="sbox"><div class="slbl">Lidar pts</div><div class="sval" id="s-pts">0</div></div>
      <div class="sbox"><div class="slbl">FPS</div><div class="sval" id="s-fps">0</div></div>
    </div>
    <div class="bottom-row-inner">
      <div class="compass">
        <div class="slbl">BEARING — LEFT ← · → RIGHT</div>
        <div class="ctrack"><div id="needle"></div></div>
        <div class="clabels"><span>−90°</span><span>0°</span><span>+90°</span></div>
      </div>
      <div class="cmdbox" id="s-cmd">NO TARGET</div>
    </div>
  </div>

  <!-- Status / motor panel -->
  <div id="status-panel">

    <div>
      <div class="sp-title">Motor</div>
      <div class="sp-row"><span>Speed</span><span class="sp-val" id="sp-spd">0%</span></div>
      <div class="sp-row"><span>Direction</span><span class="sp-val" id="sp-dir">90°</span></div>
      <div class="sp-row"><span>Turn</span><span class="sp-val" id="sp-turn">STRAIGHT</span></div>
      <div class="sp-row"><span>Source</span><span class="sp-val" id="sp-src">—</span></div>
    </div>

    <div>
      <div class="sp-title">Brain</div>
      <div class="mode-badge" id="sp-brain-mode">IDLE</div>
      <div class="sp-row" style="margin-top:6px;"><span>Front clear</span><span class="sp-val" id="sp-fc">—</span></div>
      <div class="fc-bar-bg"><div class="fc-bar-fill" id="fc-fill"></div></div>
    </div>

    <div>
      <div class="sp-title">Pickup servo</div>
      <div style="display:flex;align-items:center;gap:10px;margin-top:4px;">
        <div class="mc-toggle" id="pickup-toggle"><div class="mc-toggle-thumb"></div></div>
        <span style="font-family:var(--mono);font-size:.65rem;color:var(--dim);" id="pickup-lbl">OFF</span>
      </div>
    </div>

    <div>
      <div class="sp-title">Packet</div>
      <div class="packet-box">
        CMD=<span id="p-cmd">1</span> DIR=<span id="p-dir">90</span>°<br>
        TURN=<span id="p-turn">NONE</span> SPD=<span id="p-spd">0</span><br>
        <span id="p-bytes">0x00 0x16 0x80</span>
      </div>
    </div>

  </div>
</div>

<script>
  const streamHost = window.location.hostname;

  // ─── Camera ───
  const feed    = document.getElementById('cam-feed');
  const offline = document.getElementById('cam-offline');
  feed.onload  = () => offline.classList.remove('show');
  feed.onerror = () => { offline.classList.add('show'); setTimeout(() => { feed.src = `http://${streamHost}:8081`; }, 2000); };
  feed.src = `http://${streamHost}:8081`;

  // ─── Radar ───
  const radar = document.getElementById('radar');
  const ctx   = radar.getContext('2d');
  const W = radar.width, H = radar.height, CX = W/2, CY = H/2, R = CX - 4;
  let zoom=1.0, panX=0, panY=0, radarDragging=false, dragStart={}, lastPoints=[];
  const MAX_RANGE = 10.0;
  function mToPx(mx, my) { const sc=(R/MAX_RANGE)*zoom; return {x:CX+panX+mx*sc, y:CY+panY-my*sc}; }
  function drawRadar(points) {
    lastPoints = points;
    ctx.clearRect(0,0,W,H);
    const sc=(R/MAX_RANGE)*zoom;
    const rs=niceRingStep(MAX_RANGE/zoom), tr=Math.ceil((MAX_RANGE/zoom)/rs)+1;
    ctx.save(); ctx.beginPath(); ctx.arc(CX,CY,R,0,Math.PI*2); ctx.clip();
    const bg=ctx.createRadialGradient(CX,CY,0,CX,CY,R); bg.addColorStop(0,'#0d1a10'); bg.addColorStop(1,'#080c10');
    ctx.fillStyle=bg; ctx.fillRect(0,0,W,H);
    const o=mToPx(0,0); ctx.lineWidth=0.8;
    for(let i=1;i<=tr;i++){const rm=i*rs,rp=rm*sc; ctx.strokeStyle='#1a2d45'; ctx.beginPath(); ctx.arc(o.x,o.y,rp,0,Math.PI*2); ctx.stroke(); const lx=o.x+3,ly=o.y-rp+11; if(ly>4&&ly<H-4&&lx>4&&lx<W-4){ctx.fillStyle='#4a6080'; ctx.font='9px Share Tech Mono'; ctx.fillText(rm<1?(rm*100).toFixed(0)+'cm':rm.toFixed(rm<10?1:0)+'m',lx,ly);}}
    ctx.strokeStyle='#1a2d45'; ctx.lineWidth=0.6;
    ctx.beginPath(); ctx.moveTo(o.x,0); ctx.lineTo(o.x,H); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0,o.y); ctx.lineTo(W,o.y); ctx.stroke();
    ctx.strokeStyle='#00e5ff33'; ctx.lineWidth=1;
    ctx.beginPath(); ctx.moveTo(o.x,o.y); ctx.lineTo(o.x,o.y-R*zoom); ctx.stroke();
    points.forEach(p=>{const rad=(p.a-90)*Math.PI/180,mx=Math.cos(rad)*p.r,my=Math.sin(rad)*p.r,px=mToPx(mx,my); const g=ctx.createRadialGradient(px.x,px.y,0,px.x,px.y,4); g.addColorStop(0,'#39ff84aa'); g.addColorStop(1,'transparent'); ctx.fillStyle=g; ctx.fillRect(px.x-4,px.y-4,8,8); ctx.fillStyle='#39ff84'; ctx.fillRect(px.x-1,px.y-1,2,2);});
    ctx.shadowBlur=10; ctx.shadowColor='#00e5ff'; ctx.fillStyle='#00e5ff'; ctx.beginPath(); ctx.arc(o.x,o.y,3,0,Math.PI*2); ctx.fill(); ctx.shadowBlur=0; ctx.restore();
    ctx.fillStyle='#4a6080'; ctx.font='9px Share Tech Mono'; ctx.fillText(`x${zoom.toFixed(1)}  ${(MAX_RANGE/zoom).toFixed(1)}m`,8,H-8);
  }
  function niceRingStep(r){const s=[0.25,0.5,1,2,2.5,5,10],i=r/4; return s.reduce((p,x)=>Math.abs(x-i)<Math.abs(p-i)?x:p);}
  radar.style.cursor='grab';
  radar.addEventListener('wheel',e=>{e.preventDefault(); zoom=Math.max(0.5,Math.min(20,zoom*(e.deltaY<0?1.15:1/1.15))); drawRadar(lastPoints);},{passive:false});
  radar.addEventListener('mousedown',e=>{radarDragging=true; dragStart={x:e.clientX,y:e.clientY,px:panX,py:panY}; radar.style.cursor='grabbing';});
  window.addEventListener('mousemove',e=>{if(!radarDragging)return; panX=dragStart.px+(e.clientX-dragStart.x); panY=dragStart.py+(e.clientY-dragStart.y); drawRadar(lastPoints);});
  window.addEventListener('mouseup',()=>{radarDragging=false; radar.style.cursor='grab';});
  radar.addEventListener('dblclick',()=>{zoom=1;panX=0;panY=0; drawRadar(lastPoints);});

  // ─── Sensor polling ───
  function sv(id,text,cls){const el=document.getElementById(id); el.textContent=text; if(cls!==null) el.className='sval '+(cls||'');}
  async function fetchLidar(){try{const d=await(await fetch('/data')).json(); drawRadar(d); document.getElementById('pt-count').textContent=d.length+' pts'; sv('s-pts',d.length,null);}catch(e){}}
  async function fetchTracking(){
    try{
      const d=await(await fetch('/tracking')).json();
      sv('s-obj',d.objects,d.objects>0?'ok':'dim');
      sv('s-ang',d.angle.toFixed(1)+'°',Math.abs(d.angle)<5?'ok':'warn');
      sv('s-dist',d.distance.toFixed(2),null);
      sv('s-fps',d.actual_fps,d.actual_fps>=24?'ok':'warn');
      const cmd=document.getElementById('s-cmd');
      cmd.textContent=d.command;
      const isF=d.command==='FORWARD',isN=d.command.startsWith('NO');
      cmd.style.color=isF?'var(--ok)':isN?'var(--dim)':'var(--warn)';
      cmd.style.borderLeftColor=isF?'var(--ok)':isN?'var(--dim)':'var(--warn)';
      document.getElementById('needle').style.left=Math.max(0,Math.min(100,((d.angle+90)/180)*100))+'%';
    }catch(e){}
  }
  function sensorLoop(){fetchLidar(); fetchTracking(); setTimeout(sensorLoop,100);}
  sensorLoop();

  // ══════════════════════════════════════════
  // ─── Mode: 'off' | 'manual' | 'auto' ───
  // ══════════════════════════════════════════
  let currentMode = 'off';
  let ms = { cmd:1, direction:90, turn:0, speed:0, pickup:false };
  const FIXED_FWD = 90, FIXED_REV = 270;

  const dpBtns = {
    fwd:   document.getElementById('dp-fwd'),
    back:  document.getElementById('dp-back'),
    left:  document.getElementById('dp-left'),
    right: document.getElementById('dp-right')
  };
  const dpTurnBtns = {
    0: document.getElementById('dp-tnone'),
    1: document.getElementById('dp-tleft'),
    2: document.getElementById('dp-tright')
  };
  const dpSlider  = document.getElementById('dp-slider');
  const dpSpdLbl  = document.getElementById('dp-spd');
  const pickupTog = document.getElementById('pickup-toggle');
  const pickupLbl = document.getElementById('pickup-lbl');

  // Which direction buttons are currently physically held
  const held = { fwd: false, back: false, left: false, right: false, turnLeft: false, turnRight: false };

  // ─── Mode button styling ───
  function applyModeBtnStyle(mode) {
    document.getElementById('btn-off').className  = 'mode-btn' + (mode==='off'    ? ' active-off'  : '');
    document.getElementById('btn-man').className  = 'mode-btn' + (mode==='manual' ? ' active-man'  : '');
    document.getElementById('btn-auto').className = 'mode-btn' + (mode==='auto'   ? ' active-auto' : '');
  }

  function setManualControlsEnabled(on) {
    Object.values(dpBtns).forEach(b => b.classList.toggle('disabled', !on));
    Object.values(dpTurnBtns).forEach(b => b.classList.toggle('disabled', !on));
    dpSlider.disabled      = !on;
    dpSlider.style.opacity = on ? '' : '0.35';
    pickupTog.style.pointerEvents = on ? '' : 'none';
    pickupTog.style.opacity       = on ? '' : '0.35';
  }

  async function setMode(mode) {
    if (mode === currentMode) return;
    currentMode = mode;
    applyModeBtnStyle(mode);
    releaseAll();
    if (mode === 'off') {
      setManualControlsEnabled(false);
      await fetch('/motor/estop', {method:'POST'});
    } else if (mode === 'manual') {
      setManualControlsEnabled(true);
      await fetch('/brain', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({enable:0})});
    } else if (mode === 'auto') {
      setManualControlsEnabled(false);
      await fetch('/brain', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({enable:1})});
    }
  }

  applyModeBtnStyle('off');
  setManualControlsEnabled(false);

  // ─── Speed slider ───
  dpSlider.addEventListener('input', () => {
    dpSpdLbl.textContent = dpSlider.value;
    if (anyHeld()) sendCurrentHeld(); // update live if driving
  });

  function getSpeed() { return parseInt(dpSlider.value); }
  function anyHeld()  { return held.fwd || held.back || held.left || held.right || held.turnLeft || held.turnRight; }

  // ─── Compute and send based on what's held ───
  // direction = 0  → stopped
  // direction = 90 → forward
  //
  // left/right alone  → turn in place (direction=90, turn=1/2)
  // fwd  + right      → diagonal 45°
  // fwd  + left       → diagonal 135°
  // back + right      → diagonal 315°
  // back + left       → diagonal 225°
  // fwd  only         → 90°
  // back only         → 270°
  function sendCurrentHeld() {
    if (currentMode !== 'manual') return;
    if (!anyHeld()) {
      ms.speed     = 0;
      ms.direction = 0;
      ms.turn      = 0;
    } else {
      ms.speed = getSpeed();
      // Direction — resolved independently from turn
      if      (held.fwd  && held.right) ms.direction = 45;
      else if (held.right && held.back) ms.direction = 135;
      else if (held.back  && held.left) ms.direction = 225;
      else if (held.left  && held.fwd)  ms.direction = 315;
      else if (held.fwd)                ms.direction = 360;
      else if (held.right)              ms.direction = 90;
      else if (held.back)               ms.direction = 180;
      else if (held.left)               ms.direction = 270;
      else                              ms.direction = 0;
      // Turn — resolved independently, stacks with any direction
      if      (held.turnRight)          ms.turn = 2;
      else if (held.turnLeft)           ms.turn = 1;
      else                              ms.turn = 0;
    }
    syncDisplay();
    sendMotor();
  }

  function releaseAll() {
    Object.keys(held).forEach(k => held[k] = false);
    Object.values(dpBtns).forEach(b => b.classList.remove('pressed'));
    ms.speed = 0; ms.direction = 0; ms.turn = 0;
    syncDisplay();
    if (currentMode === 'manual') sendMotor();
  }

  // ─── D-pad: momentary — hold = move, release = stop ───
  Object.entries(dpBtns).forEach(([dir, btn]) => {
    function press(e) {
      e.preventDefault();
      if (currentMode !== 'manual') return;
      held[dir] = true;
      btn.classList.add('pressed');
      sendCurrentHeld();
    }
    function release() {
      held[dir] = false;
      btn.classList.remove('pressed');
      sendCurrentHeld();
    }
    btn.addEventListener('mousedown',   press);
    btn.addEventListener('touchstart',  press,   {passive:false});
    btn.addEventListener('mouseup',     release);
    btn.addEventListener('mouseleave',  release);
    btn.addEventListener('touchend',    release);
    btn.addEventListener('touchcancel', release);
  });

  // ─── Turn buttons: sticky toggle ───
  function setTurn(val) {
    if (currentMode !== 'manual') return;
    ms.turn = val;
    Object.entries(dpTurnBtns).forEach(([v,b]) => b.classList.toggle('active', parseInt(v)===val));
    syncDisplay();
    if (anyHeld()) sendMotor();
  }
  Object.entries(dpTurnBtns).forEach(([v,b]) => b.addEventListener('click', () => setTurn(parseInt(v))));

  // ─── Pickup toggle ───
  pickupTog.addEventListener('click', () => {
    if (currentMode !== 'manual') return;
    ms.pickup = !ms.pickup;
    pickupTog.classList.toggle('on', ms.pickup);
    pickupLbl.textContent = ms.pickup ? 'ON' : 'OFF';
    syncDisplay(); sendMotor();
  });

  // ─── Keyboard: arrows + A/D = momentary hold, Space = off ───
  // Arrow left/right → pure direction (no turn)
  // A / D            → turn in place only (no direction strafe)
  const keysHeld = {};
  const arrowToDir = { ArrowUp:'fwd', ArrowDown:'back', ArrowLeft:'left', ArrowRight:'right' };

  window.addEventListener('keydown', e => {
    if (e.target.tagName === 'INPUT') return;
    const k = e.key;
    if (['ArrowUp','ArrowDown','ArrowLeft','ArrowRight','a','A','d','D',' '].includes(k)) e.preventDefault();
    if (k === ' ') { setMode('off'); return; }
    if (keysHeld[k]) return;
    keysHeld[k] = true;

    if (k==='a'||k==='A') { held.turnLeft  = true;  sendCurrentHeld(); return; }
    if (k==='d'||k==='D') { held.turnRight = true;  sendCurrentHeld(); return; }

    const dir = arrowToDir[k];
    if (dir) {
      held[dir] = true;
      dpBtns[dir].classList.add('pressed');
      sendCurrentHeld();
    }
  });

  window.addEventListener('keyup', e => {
    const k = e.key;
    delete keysHeld[k];
    if (k==='a'||k==='A') { held.turnLeft  = false; sendCurrentHeld(); return; }
    if (k==='d'||k==='D') { held.turnRight = false; sendCurrentHeld(); return; }
    const dir = arrowToDir[k];
    if (dir) {
      held[dir] = false;
      dpBtns[dir].classList.remove('pressed');
      sendCurrentHeld();
    }
  });


  // ─── Display sync ───
  function syncDisplay() {
    dpSpdLbl.textContent = ms.speed;
    Object.entries(dpTurnBtns).forEach(([v,b]) => b.classList.toggle('active', parseInt(v)===ms.turn));
    document.getElementById('sp-spd').textContent  = ms.speed + '%';
    document.getElementById('sp-dir').textContent  = ms.direction + '°';
    document.getElementById('sp-turn').textContent = ['STRAIGHT','LEFT','RIGHT'][ms.turn];
    document.getElementById('p-cmd').textContent   = ms.cmd;
    document.getElementById('p-dir').textContent   = ms.direction;
    document.getElementById('p-turn').textContent  = ['NONE','LEFT','RIGHT'][ms.turn];
    document.getElementById('p-spd').textContent   = ms.speed;
    const b = buildPacket(ms.cmd, ms.direction, ms.turn, ms.speed, ms.pickup);
    document.getElementById('p-bytes').textContent = b.map(n=>'0x'+n.toString(16).toUpperCase().padStart(2,'0')).join(' ');
  }

  function buildPacket(cmd,dir,turn,speed,pickup) {
    let p=0;
    if(cmd) p|=(1<<19); p|=((dir&0x1FF)<<10); p|=((turn&0x03)<<8); p|=((speed&0x7F)<<1); if(pickup) p|=0x01;
    return [(p>>16)&0xFF,(p>>8)&0xFF,p&0xFF];
  }

  async function sendMotor() {
    if (currentMode !== 'manual') return;
    try {
      await fetch('/motor', {
        method:'POST', headers:{'Content-Type':'application/json'},
        body: JSON.stringify({cmd:ms.cmd, direction:ms.direction, turn:ms.turn, speed:ms.speed, pickup:ms.pickup?1:0})
      });
    } catch(e) {}
  }


  // ─── Poll /motor/state for live status panel ───
  async function fetchMotorState() {
    try {
      const d = await (await fetch('/motor/state')).json();

      // Sync mode if server disagrees (e.g. estop from another client)
      const serverBrain = d.brain_enabled===1;
      if (serverBrain && currentMode!=='auto')   { currentMode='auto';   applyModeBtnStyle('auto');   setManualControlsEnabled(false); }
      if (!serverBrain && d.estopped===1 && currentMode!=='off') { currentMode='off'; applyModeBtnStyle('off'); setManualControlsEnabled(false); }

      // Status panel
      document.getElementById('sp-spd').textContent  = d.speed + '%';
      document.getElementById('sp-dir').textContent  = d.direction + '°';
      document.getElementById('sp-turn').textContent = ['STRAIGHT','LEFT','RIGHT'][d.turn] || '—';
      document.getElementById('sp-src').textContent  = (d.source||'—').toUpperCase();

      // Brain badge
      const bm = (d.brain_mode||'IDLE').toUpperCase();
      const badge = document.getElementById('sp-brain-mode');
      badge.textContent = bm;
      badge.className   = 'mode-badge ' + bm.toLowerCase();

      // Header brain strip
      const bdot = document.getElementById('hdr-bdot');
      const modeClass = bm.toLowerCase();
      bdot.className = 'bdot ' + (serverBrain ? modeClass : '');
      document.getElementById('hdr-mode-lbl').textContent = serverBrain ? bm : (currentMode==='off'?'OFF':'MANUAL');
      document.getElementById('hdr-reason').textContent   = serverBrain ? (d.brain_reason||'') : '';

      // Front clear bar
      const fc = parseFloat(d.front_clear);
      const fcEl = document.getElementById('sp-fc');
      const fcFill = document.getElementById('fc-fill');
      if (fc < 0) {
        fcEl.textContent = '>10m'; fcEl.className='sp-val ok'; fcFill.style.width='100%'; fcFill.style.background='var(--ok)';
      } else {
        fcEl.textContent = (fc*100).toFixed(0)+'cm';
        const pct = Math.min(100, (fc/2)*100);
        fcFill.style.width = pct+'%';
        fcFill.style.background = fc<0.6?'var(--warn)':'var(--ok)';
        fcEl.className = 'sp-val '+(fc<0.6?'warn':'ok');
      }
    } catch(e){}
  }
  setInterval(fetchMotorState, 400);

  syncDisplay();
</script>
</body>
</html>
<!-- " -->
)html";

// ---------- main ----------
int main() {
    signal(SIGINT, signalHandler);

    // --- Lidar ---
    LidarController lidar("/dev/ttyUSB0", FLIP_LIDAR);
    if (!lidar.initialize()) {
        std::cerr << "Failed to initialize lidar\n";
        return -1;
    }

    // --- Camera ---
    CamController cam(0, CAMWIDTH, CAMHEIGHT);
    cam.addTrackedColor("red",    0,   10,  100, 100, 170, 180, 100, 100);
    cam.addTrackedColor("purple", 130, 160, 100, 100, 130, 160, 100, 100);
    cam.setFlip(FLIP_CAMERA);
    cam.setTrackingStrategy(CamController::TrackingStrategy::LARGEST);
    cam.setMinArea(MIN_AREA);
    cam.setDeadOnThreshold(DEAD_ON_THRESH);
    if (!cam.initialize()) {
        std::cerr << "Failed to initialize camera\n";
        return -1;
    }
    cam.enableStreaming(true, STREAM_PORT);
    std::cout << "MJPEG stream : http://<your-pi-ip>:" << STREAM_PORT << "\n";

    // --- Motor (wire into MotorBus) ---
    MotorController motor("/dev/ttyUSB1");
    if (!motor.isOpen()) {
        std::cerr << "Warning: MotorController failed to open /dev/ttyUSB1 — motor commands will be ignored\n";
    } else {
        motor.eStop();
    }
    global_motor_bus.hw = &motor;   // <-- bus now owns the hardware ref

    // --- Threads ---
    std::thread t_lidar(lidarThread,  std::ref(lidar));
    std::thread t_cam  (cameraThread, std::ref(cam));
    std::thread t_brain(brainThread);

    // --- Crow dashboard ---
    crow::SimpleApp app;

    CROW_ROUTE(app, "/")([&](){ return DASHBOARD_HTML; });

    // GET /data — lidar scan
    CROW_ROUTE(app, "/data")([&](){
        crow::json::wvalue x;
        std::lock_guard<std::mutex> lk(scan_mutex);
        for (size_t i = 0; i < global_scan.size(); i++) {
            x[i]["a"] = global_scan[i].angle;
            x[i]["r"] = global_scan[i].range;
        }
        return x;
    });

    // GET /tracking — camera tracking state
    CROW_ROUTE(app, "/tracking")([&](){
        crow::json::wvalue x;
        std::lock_guard<std::mutex> lk(tracking_mutex);
        x["objects"]    = global_tracking.objectCount;
        x["angle"]      = global_tracking.angle;
        x["distance"]   = global_tracking.distance;
        x["command"]    = global_tracking.command;
        x["actual_fps"] = global_tracking.actual_fps;
        return x;
    });

    // POST /motor — manual drive command { cmd, direction, turn, speed, pickup }
    //   Only accepted when brain is disabled; rejected otherwise.
    CROW_ROUTE(app, "/motor").methods(crow::HTTPMethod::POST)([&](const crow::request& req){
        {
            std::lock_guard<std::mutex> lk(brain_mutex);
            if (global_brain.enabled)
                return crow::response(409, "brain active — disable autonomous mode first");
        }
        auto body = crow::json::load(req.body);
        if (!body) return crow::response(400, "bad json");

        MotorCommand cmd;
        cmd.cmd_enable = body["cmd"].i()       != 0;
        cmd.direction  = body["direction"].i();
        cmd.turn       = body["turn"].i();
        cmd.speed      = body["speed"].i();
        cmd.pickup     = body["pickup"].i()    != 0;

        global_motor_bus.drive(cmd, "manual");
        return crow::response(200, "ok");
    });

    // POST /motor/estop — emergency stop (always accepted, kills brain too)
    CROW_ROUTE(app, "/motor/estop").methods(crow::HTTPMethod::POST)([&](){
        {
            std::lock_guard<std::mutex> lk(brain_mutex);
            global_brain.enabled = false;
            global_brain.mode    = "IDLE";
        }
        global_motor_bus.eStop("estop-btn");
        return crow::response(200, "ok");
    });

    // GET /motor/state — current motor state + brain state
    CROW_ROUTE(app, "/motor/state")([&](){
        crow::json::wvalue x;
        auto cmd = global_motor_bus.getLastCmd();
        {
            std::lock_guard<std::mutex> lk(global_motor_bus.mtx);
            x["cmd"]       = cmd.cmd_enable ? 1 : 0;
            x["direction"] = cmd.direction;
            x["turn"]      = cmd.turn;
            x["speed"]     = cmd.speed;
            x["pickup"]    = cmd.pickup ? 1 : 0;
            x["estopped"]  = global_motor_bus.estopped ? 1 : 0;
            x["source"]    = global_motor_bus.source;
        }
        {
            std::lock_guard<std::mutex> lk(brain_mutex);
            x["brain_enabled"] = global_brain.enabled ? 1 : 0;
            x["brain_mode"]    = global_brain.mode;
            x["brain_reason"]  = global_brain.reason;
            x["front_clear"]   = global_brain.front_clear;
        }
        return x;
    });

    // POST /brain — { "enable": 0|1 }  toggle autonomous mode
    CROW_ROUTE(app, "/brain").methods(crow::HTTPMethod::POST)([&](const crow::request& req){
        auto body = crow::json::load(req.body);
        if (!body) return crow::response(400, "bad json");
        bool enable = body["enable"].i() != 0;
        {
            std::lock_guard<std::mutex> lk(brain_mutex);
            global_brain.enabled = enable;
            global_brain.mode    = enable ? "SEEK" : "IDLE";
            global_brain.reason  = enable ? "just enabled" : "disabled by user";
        }
        if (!enable) {
            // Hand back to manual — send a safe stop
            MotorCommand stop;
            stop.speed = 0;
            global_motor_bus.drive(stop, "manual");
        } else {
            global_motor_bus.resume();
        }
        std::cout << "[brain] autonomous mode " << (enable ? "ENABLED" : "DISABLED") << "\n";
        return crow::response(200, "ok");
    });

    std::cout << "Dashboard    : http://<your-pi-ip>:" << WEB_PORT << "\n";
    app.port(WEB_PORT).multithreaded().run();

    // Shutdown
    running = false;
    {
        std::lock_guard<std::mutex> lk(brain_mutex);
        global_brain.enabled = false;
    }
    global_motor_bus.eStop("shutdown");
    t_lidar.join();
    t_cam.join();
    t_brain.join();
    cam.release();

    return 0;
}
