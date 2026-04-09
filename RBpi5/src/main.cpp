/**
 * @file main.cpp
 * @brief Main control software for the autonomously robot.
 * * This file sets up the hardware controllers (Lidar, Camera, Motor), initializes
 * the internal state map, spawns sensor and autonomous brain threads, and serves 
 * a web-based dashboard using Crow for telemetry and manual remote control.
 */

#include "LidarController.h"
#include "CamController.h"
#include "MotorController.h"
#include "MapController.h"
#include "Config.h"
#include "Brain.h"
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

/// Global configuration loaded from config.txt
RobotConfig CFG;

/// Atomic flag to control the main execution loop and thread lifespans
std::atomic<bool> running{true};

/// @name Shared Sensor State
/// Mutex-protected global structures for sensor telemetry.
///@{

/// Latest full 360-degree Lidar scan
std::vector<LidarPoint> global_scan;
/// Mutex protecting global_scan
std::mutex              scan_mutex;
/**
 * @brief Represents the current vision tracking state.
 */
struct TrackingState {
    double      angle        = 0.0;         ///< Bearing angle to target
    double      distance     = 0.0;         ///< Distance to target
    int         objectCount  = 0;           ///< Number of detected objects
    std::string command      = "NO TARGET"; ///< String representation of current tracking command
    int         actual_fps   = 0;           ///< Measured camera processing frames per second

    double      targetPixelX = -1.0;        ///< Centre-X of chosen target in pixel space (-1.0 if none)
    double      targetPixelY = -1.0;        ///< Centre-Y of chosen target in pixel space (-1.0 if none)
    double      targetArea   = 0.0;         ///< Contour area of the target in pixels squared
    int         frameWidth   = 640;         ///< Width of the camera frame
};

/// Global instance of tracking state
TrackingState global_tracking;
/// Mutex protecting global_tracking
std::mutex    tracking_mutex;
///@}

/// @name Map State
///@{
/// Global SLAM map controller instance
MapController global_map;
/// Mutex protecting global_map
std::mutex    map_mutex;
///@}


#ifndef MOTOR_COMMAND_DEFINED
#define MOTOR_COMMAND_DEFINED
/**
 * @brief Encapsulates a movement command for the motor controller.
 */
struct MotorCommand {
    bool   cmd_enable = false;  ///< True if motors are enabled, false triggers an emergency stop
    int    direction  = 360;    ///< Movement direction in degrees (360 = forward)
    int    turn       = 0;     ///< Turn modifier: 0=Straight, 1=Left, 2=Right
    int    speed      = 0;     ///< Speed percentage (0-100)
    bool   pickup     = false; ///< True to engage the pickup servo mechanism
};
#endif

/**
 * @brief Thread-safe bus for sending commands to the hardware motor controller.
 */
struct MotorBus {
    MotorController* hw      = nullptr; ///< Pointer to the active hardware controller
    mutable std::mutex      mtx;               ///< Mutex protecting bus state

    MotorCommand            last_cmd;          ///< The most recently issued command
    bool                    estopped = false;  ///< True if the system is currently in E-Stop
    std::string             source   = "manual";///< Source identifier of the last command
    uint64_t                seq      = 0;      ///< Sequence number incremented on every drive call

    /**
     * @brief Dispatches a motor command to the hardware.
     * @param cmd The motor command payload.
     * @param src String identifying the source of the command (e.g., "manual", "brain").
     */
    void drive(const MotorCommand& cmd, const std::string& src) {
        std::lock_guard<std::mutex> lk(mtx);
        last_cmd = cmd;
        source   = src;
        seq++;
        if (!hw || !hw->isOpen()) return;
        if (!cmd.cmd_enable) { hw->eStop(); return; }
        MotorController::TurnDirection td = MotorController::TurnDirection::NONE;
        if (cmd.turn == 1) td = MotorController::TurnDirection::LEFT;
        if (cmd.turn == 2) td = MotorController::TurnDirection::RIGHT;
        hw->drive(cmd.direction, td, cmd.speed, cmd.pickup);
    }

    /**
     * @brief Triggers an emergency stop, halting all motor movement.
     * @param src Identifier for the source triggering the E-Stop.
     */
    void eStop(const std::string& src = "manual") {
        std::lock_guard<std::mutex> lk(mtx);
        estopped        = true;
        last_cmd.speed  = 0;
        source          = src;
        if (hw && hw->isOpen()) hw->eStop();
    }

    /**
     * @brief Clears the E-Stop state.
     */
    void resume() {
        std::lock_guard<std::mutex> lk(mtx);
        estopped = false;
    }

    /**
     * @brief Retrieves a copy of the last dispatched motor command.
     * @return MotorCommand structure.
     */
    MotorCommand getLastCmd() const {
        std::lock_guard<std::mutex> lk(mtx);
        return last_cmd;
    }
};

/// Global motor bus instance
MotorBus global_motor_bus;


/**
 * @brief Holds telemetry data regarding the autonomous logic system.
 */
struct BrainState {
    bool        enabled     = false; ///< True if autonomous mode is active
    std::string mode        = "IDLE";///< Current behavior tree mode 
    std::string reason      = "";    ///< Debug string explaining the current mode
    float       front_clear = 0.0f;  ///< Free distance directly in front of the robot (meters)
};

/// Global brain state instance
BrainState  global_brain;
/// Mutex protecting global_brain
std::mutex  brain_mutex;


/**
 * @brief Intercepts OS signals (e.g., SIGINT) to ensure graceful shutdown.
 * @param signum OS signal number.
 */
void signalHandler(int) {
    std::cout << "\nShutting down..." << std::endl;
    running = false;
}

/**
 * @brief Continuous background thread for fetching Lidar scans.
 * @param lidar Reference to the initialized LidarController.
 */
void lidarThread(LidarController& lidar) {
    while (running) {
        auto scan = lidar.getLatestScan();
        if (!scan.empty()) {
            {
                std::lock_guard<std::mutex> lk(scan_mutex);
                global_scan = scan;
            }
            bool brainOn;
            { std::lock_guard<std::mutex> lk(brain_mutex); brainOn = global_brain.enabled; }
            if (brainOn) global_map.update(scan);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

/**
 * @brief Continuous background thread for capturing frames and performing CV tracking.
 * @param cam Reference to the initialized CamController.
 */
void cameraThread(CamController& cam) {
    const long frame_us = 1000000 / CFG.camFps;
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

        CamController::Direction dir     = cam.getDirection();
        auto                     objects = cam.getDetectedObjects();

        {
            std::lock_guard<std::mutex> lk(tracking_mutex);
            global_tracking.angle       = dir.angle;
            global_tracking.distance    = dir.distance;
            global_tracking.objectCount = dir.objectCount;
            global_tracking.command     = dir.command;
            global_tracking.frameWidth  = CFG.camWidth;

            global_tracking.targetPixelX = -1.0;
            global_tracking.targetPixelY = -1.0;
            global_tracking.targetArea   = 0.0;
            for (const auto& obj : objects) {
                if (obj.area > global_tracking.targetArea) {
                    global_tracking.targetArea   = obj.area;
                    global_tracking.targetPixelX = obj.center.x;
                    global_tracking.targetPixelY = obj.center.y;
                }
            }
        }

        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - t0).count();
        if (long s = frame_us - elapsed; s > 0) usleep(s);
    }
}

/**
 * @brief Autonomous decision loop that processes sensor data and emits motor commands.
 * * Implements a prioritized state machine:
 * 1. E-stop active          -> IDLE
 * 2. Obstacle < clearDist   -> AVOID
 * 3. Target close (area >= brain_collect_dist) -> COLLECT
 * Phase A: sidestep left until cup centre is between purple lines
 * Phase B: drive straight forward with pickup=true
 * 4. Target visible (far)   -> CHASE
 * 5. No target              -> SEEK
 */

void brainThread() {
    const auto period = std::chrono::milliseconds(1000 / CFG.brainHz);

    Brain brain(CFG);

    while (running) {
        auto t0 = std::chrono::steady_clock::now();

        {
            std::lock_guard<std::mutex> lk(brain_mutex);
            if (!global_brain.enabled) {
                std::this_thread::sleep_until(t0 + period);
                continue;
            }
        }

        
        std::vector<LidarPoint> scan;
        { std::lock_guard<std::mutex> lk(scan_mutex);     scan = global_scan; }

        TrackingState ts;
        { std::lock_guard<std::mutex> lk(tracking_mutex); ts = global_tracking; }

        bool estopped;
        { std::lock_guard<std::mutex> lk(global_motor_bus.mtx); estopped = global_motor_bus.estopped; }

        
        TrackingSnapshot snap;
        snap.angle        = ts.angle;
        snap.distance     = ts.distance;
        snap.objectCount  = ts.objectCount;
        snap.command      = ts.command;
        snap.targetPixelX = ts.targetPixelX;
        snap.targetPixelY = ts.targetPixelY;
        snap.targetArea   = ts.targetArea;
        snap.frameWidth   = ts.frameWidth;

        
        BrainOutput out = brain.tick(estopped, scan, snap);

        
        global_motor_bus.drive(out.cmd, "brain");

        {
            std::lock_guard<std::mutex> lk(brain_mutex);
            global_brain.mode        = out.mode;
            global_brain.reason      = out.reason;
            global_brain.front_clear = out.frontClear;
        }

        std::this_thread::sleep_until(t0 + period);
    }
}

/**
 * @brief Raw HTML string containing the frontend web dashboard UI and client-side logic.
 */
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
    --collect: #cc00ff;
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
  .bdot.seek    { background: var(--warn);    box-shadow: 0 0 5px var(--warn); }
  .bdot.chase   { background: var(--ok);      box-shadow: 0 0 5px var(--ok); }
  .bdot.avoid   { background: var(--warn);    box-shadow: 0 0 5px var(--warn); }
  .bdot.collect { background: var(--collect); box-shadow: 0 0 5px var(--collect); }
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
  .sval.ok      { color: var(--ok);      text-shadow: 0 0 8px rgba(57,255,132,.35); }
  .sval.warn    { color: var(--warn);    text-shadow: 0 0 8px rgba(255,61,90,.35); }
  .sval.dim     { color: var(--dim);     text-shadow: none; }
  .sval.collect { color: var(--collect); text-shadow: 0 0 8px rgba(204,0,255,.35); }
  .bottom-row-inner { display: grid; grid-template-columns: 1fr auto; gap: 8px; align-items: stretch; }
  .compass { background: var(--surface); border: 1px solid var(--border); border-radius: 3px; padding: 8px 12px; display: flex; flex-direction: column; gap: 6px; }
  .ctrack  { width: 100%; height: 5px; background: var(--border); border-radius: 3px; position: relative; }
  #needle  { position: absolute; width: 11px; height: 11px; background: var(--accent); border-radius: 50%; top: -3px; left: 50%; transform: translateX(-50%); box-shadow: 0 0 8px var(--accent); transition: left .12s ease; }
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
  .sp-val.ok      { color: var(--ok); }
  .sp-val.warn    { color: var(--warn); }

  /* brain mode badge */
  .mode-badge {
    font-family: var(--mono); font-size: .7rem; letter-spacing: .06em;
    padding: 4px 10px; border-radius: 2px; border: 1px solid var(--border);
    color: var(--dim); background: var(--bg);
    transition: color .2s, border-color .2s, background .2s;
    text-align: center;
  }
  .mode-badge.seek    { color: var(--warn);    border-color: var(--warn);    background: rgba(255,61,90,.08); }
  .mode-badge.chase   { color: var(--ok);      border-color: var(--ok);      background: rgba(57,255,132,.08); }
  .mode-badge.avoid   { color: var(--accent);  border-color: var(--accent);  background: rgba(0,229,255,.08); }
  .mode-badge.collect { color: var(--collect); border-color: var(--collect); background: rgba(204,0,255,.08); }

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

  /* ── Map panel ── */
  #map-panel {
    background: var(--bg);
    border-top: 1px solid var(--border);
    padding: 10px 18px 14px;
    display: flex;
    flex-direction: column;
    gap: 8px;
    flex-shrink: 0;
  }
  #map-toolbar {
    display: flex;
    align-items: center;
    gap: 12px;
  }
  #map-canvas {
    display: block;
    background: #060a0d;
    border: 1px solid var(--border);
    border-radius: 3px;
    cursor: grab;
    width: 100%;
    height: 340px;
  }
  .map-legend {
    display: flex;
    gap: 14px;
    font-family: var(--mono);
    font-size: .55rem;
    color: var(--dim);
    letter-spacing: .08em;
  }
  .map-legend span { display: flex; align-items: center; gap: 5px; }
  .leg-dot { width: 8px; height: 8px; border-radius: 50%; flex-shrink: 0; }
  #map-pose-lbl {
    font-family: var(--mono);
    font-size: .6rem;
    color: var(--dim);
    margin-left: auto;
  }
  .map-btn {
    font-family: var(--mono);
    font-size: .58rem;
    letter-spacing: .07em;
    padding: 4px 12px;
    background: var(--bg);
    border: 1px solid var(--border);
    border-radius: 2px;
    color: var(--dim);
    cursor: pointer;
    transition: color .1s, border-color .1s;
  }
  .map-btn:hover { color: var(--accent); border-color: var(--accent); }
  .map-btn.danger:hover { color: var(--warn); border-color: var(--warn); }
</style>
</head>
<body>

<header>
  <h1>// SENTINEL</h1>

  <div class="mode-group">
    <button class="mode-btn active-off" id="btn-off"  onclick="setMode('off')">&#9632; OFF</button>
    <button class="mode-btn"            id="btn-man"  onclick="setMode('manual')">&#9654; MANUAL</button>
    <button class="mode-btn"            id="btn-auto" onclick="setMode('auto')">&#9681; AUTO</button>
  </div>

  <div class="brain-strip">
    <div class="bdot" id="hdr-bdot"></div>
    <span id="hdr-mode-lbl">IDLE</span>
    <span id="hdr-reason" style="color:var(--dim);opacity:.7;max-width:180px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;"></span>
  </div>

  <span id="pill">&#9679; LIVE</span>
</header>

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

  <div id="dpad-panel">
    <span class="lbl">DRIVE CONTROL</span>

    <div style="width:100%;">
      <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:4px;">
        <span class="lbl" style="align-self:auto;">SPEED</span>
        <span style="font-family:var(--mono);font-size:.75rem;color:var(--accent);" id="dp-spd">0</span><span style="font-family:var(--mono);font-size:.55rem;color:var(--dim);">%</span>
      </div>
      <input type="range" id="dp-slider" min="0" max="100" value="50" step="1" style="width:100%;accent-color:var(--accent);">
    </div>

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

<div id="bottom-row">

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

<div id="map-panel">
  <div id="map-toolbar">
    <span class="lbl">SLAM MAP — KISS-ICP</span>
    <button class="map-btn" onclick="mapFitView()">FIT</button>
    <button class="map-btn" onclick="mapResetZoom()">RESET VIEW</button>
    <button class="map-btn danger" onclick="resetMap()">CLEAR MAP</button>
    <div class="map-legend">
      <span><div class="leg-dot" style="background:#1a3a1a;border:1px solid #39ff84;"></div>FREE</span>
      <span><div class="leg-dot" style="background:#3a1a1a;border:1px solid #ff3d5a;"></div>OCCUPIED</span>
      <span><div class="leg-dot" style="background:#00e5ff;"></div>ROBOT</span>
      <span><div class="leg-dot" style="background:#ffaa00;"></div>TRAIL</span>
    </div>
    <span id="map-pose-lbl">x: 0.00  y: 0.00  θ: 0°</span>
  </div>
  <canvas id="map-canvas"></canvas>
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
  async function fetchLidar(){
    try{const d=await(await fetch('/data')).json(); drawRadar(d); document.getElementById('pt-count').textContent=d.length+' pts'; sv('s-pts',d.length,null);}catch(e){}
    // In manual mode the radar is just for situational awareness — poll slowly
    // to keep the HTTP thread pool free for motor commands.
    setTimeout(fetchLidar, currentMode === 'manual' ? 1500 : 500);
  }
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
    // In manual mode tracking data isn't driving decisions — poll slowly.
    setTimeout(fetchTracking, currentMode === 'manual' ? 1000 : 300);
  }
  // Each fetch waits for response before scheduling next — no pile-up
  fetchLidar();
  fetchTracking();

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

  const held = { fwd: false, back: false, left: false, right: false, turnLeft: false, turnRight: false };

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

  function setMode(mode) {
    if (mode === currentMode) return;
    currentMode = mode;
    applyModeBtnStyle(mode);
    releaseAll();
    if (mode === 'off') {
      setManualControlsEnabled(false);
      fetch('/motor/estop', {method:'POST'}).catch(()=>{});
    } else if (mode === 'manual') {
      setManualControlsEnabled(true);
      // Fire-and-forget — don't await so the controls are live immediately
      fetch('/brain', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({enable:0})}).catch(()=>{});
    } else if (mode === 'auto') {
      setManualControlsEnabled(false);
      fetch('/brain', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({enable:1})}).catch(()=>{});
    }
  }

  applyModeBtnStyle('off');
  setManualControlsEnabled(false);

  // ─── Speed slider ───
  dpSlider.addEventListener('input', () => {
    dpSpdLbl.textContent = dpSlider.value;
    if (anyHeld()) sendCurrentHeld();
  });

  function getSpeed() { return parseInt(dpSlider.value); }
  function anyHeld()  { return held.fwd || held.back || held.left || held.right || held.turnLeft || held.turnRight; }

  function sendCurrentHeld() {
    if (currentMode !== 'manual') return;
    if (!anyHeld()) {
      ms.speed     = 0;
      ms.direction = 0;
      ms.turn      = 0;
    } else {
      ms.speed = getSpeed();
      if      (held.fwd  && held.right) ms.direction = 45;
      else if (held.right && held.back) ms.direction = 135;
      else if (held.back  && held.left) ms.direction = 225;
      else if (held.left  && held.fwd)  ms.direction = 315;
      else if (held.fwd)                ms.direction = 360;
      else if (held.right)              ms.direction = 90;
      else if (held.back)               ms.direction = 180;
      else if (held.left)               ms.direction = 270;
      else                              ms.direction = 0;
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
    sendMotor();
  }

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

  function setTurn(val) {
    if (currentMode !== 'manual') return;
    ms.turn = val;
    Object.entries(dpTurnBtns).forEach(([v,b]) => b.classList.toggle('active', parseInt(v)===val));
    syncDisplay();
  }
  Object.entries(dpTurnBtns).forEach(([v,b]) => b.addEventListener('click', () => setTurn(parseInt(v))));

  pickupTog.addEventListener('click', () => {
    if (currentMode !== 'manual') return;
    ms.pickup = !ms.pickup;
    pickupTog.classList.toggle('on', ms.pickup);
    pickupLbl.textContent = ms.pickup ? 'ON' : 'OFF';
    syncDisplay();
  });

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

// ─── Motor WebSocket — persistent connection for low-latency manual control ──
let motorWs = null;
let motorWsReady = false;

function openMotorWs() {
  // Dedicated WS port (ws_port in config.txt) — isolated from HTTP/MJPEG traffic.
  const url = `ws://${window.location.hostname}:8082/ws/motor`;
  motorWs = new WebSocket(url);
  motorWs.onopen  = () => { motorWsReady = true;  console.log('[WS] motor connected'); };
  motorWs.onclose = () => { motorWsReady = false; console.log('[WS] motor closed — reconnecting in 1s'); setTimeout(openMotorWs, 1000); };
  motorWs.onerror = () => { motorWsReady = false; motorWs.close(); };
}
openMotorWs();

function sendMotor() {
  const payload = JSON.stringify({
    cmd:       currentMode === 'off' ? 0 : ms.cmd,
    direction: ms.direction,
    turn:      ms.turn,
    speed:     currentMode === 'off' ? 0 : ms.speed,
    pickup:    ms.pickup ? 1 : 0
  });
  if (motorWsReady && motorWs && motorWs.readyState === WebSocket.OPEN) {
    motorWs.send(payload);   // <5 ms — persistent connection, no handshake
  } else {
    // Fallback to HTTP if WebSocket isn't up yet
    fetch('/motor', { method:'POST', headers:{'Content-Type':'application/json'}, body: payload }).catch(()=>{});
  }
}

  // ─── Poll /motor/state for live status panel ───
  async function fetchMotorState() {
    try {
      const d = await (await fetch('/motor/state')).json();

      const serverBrain = d.brain_enabled===1;
      if (serverBrain && currentMode!=='auto')   { currentMode='auto';   applyModeBtnStyle('auto');   setManualControlsEnabled(false); }
      if (!serverBrain && d.estopped===1 && currentMode!=='off') { currentMode='off'; applyModeBtnStyle('off'); setManualControlsEnabled(false); }

      document.getElementById('sp-spd').textContent  = d.speed + '%';
      document.getElementById('sp-dir').textContent  = d.direction + '°';
      document.getElementById('sp-turn').textContent = ['STRAIGHT','LEFT','RIGHT'][d.turn] || '—';
      document.getElementById('sp-src').textContent  = (d.source||'—').toUpperCase();

      // Brain badge — now also handles COLLECT (purple)
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
    setTimeout(fetchMotorState, 800);  // ~1.2 Hz — wait for response first
  }
  fetchMotorState();

  syncDisplay();

  // ══════════════════════════════════════════════════════════════════
  // ─── SLAM Map ───
  // ══════════════════════════════════════════════════════════════════
  const mapCanvas = document.getElementById('map-canvas');
  const mapCtx    = mapCanvas.getContext('2d');

  let mapZoom = 40;
  let mapPanX = 0;
  let mapPanY = 0;
  let mapDragging = false;
  let mapDragStart = {};

  let cachedGrid   = null;
  let cachedBitmap = null;

  function resizeMapCanvas() {
    mapCanvas.width  = mapCanvas.offsetWidth;
    mapCanvas.height = mapCanvas.offsetHeight;
  }
  resizeMapCanvas();
  window.addEventListener('resize', () => { resizeMapCanvas(); drawMap(); });

  function w2c(wx, wy) {
    const cx = mapCanvas.width  / 2 + mapPanX;
    const cy = mapCanvas.height / 2 + mapPanY;
    return { x: cx + wx * mapZoom, y: cy - wy * mapZoom };
  }

  function drawMap(data) {
    const W = mapCanvas.width, H = mapCanvas.height;
    mapCtx.clearRect(0, 0, W, H);

    mapCtx.fillStyle = '#060a0d';
    mapCtx.fillRect(0, 0, W, H);

    mapCtx.strokeStyle = '#1a2d45';
    mapCtx.lineWidth   = 0.5;
    const gridStep = mapZoom;
    const cx0 = (W/2 + mapPanX) % gridStep;
    const cy0 = (H/2 + mapPanY) % gridStep;
    for (let x = cx0; x < W; x += gridStep) { mapCtx.beginPath(); mapCtx.moveTo(x,0); mapCtx.lineTo(x,H); mapCtx.stroke(); }
    for (let y = cy0; y < H; y += gridStep) { mapCtx.beginPath(); mapCtx.moveTo(0,y); mapCtx.lineTo(W,y); mapCtx.stroke(); }

    if (!data) return;

    if (data.grid && data.grid.rle && data.grid.width > 0) {
      const g   = data.grid;
      const res = g.res;
      const cpx = mapZoom * res;
      const o = w2c(g.originX, g.originY + g.height * res);

      for (let ri = 0, ci = 0, row = 0, col = 0; ri < g.rle.length; ri++) {
        const val = g.rle[ri].v;
        let   cnt = g.rle[ri].n;
        while (cnt-- > 0) {
          if (val !== 0) {
            const px = o.x + col * cpx;
            const py = o.y + row * cpx;
            mapCtx.fillStyle = val === 2 ? 'rgba(255,61,90,0.55)' : 'rgba(57,255,132,0.12)';
            mapCtx.fillRect(px, py, cpx + 0.5, cpx + 0.5);
          }
          col++;
          if (col >= g.width) { col = 0; row++; }
        }
      }
    }

    if (data.cloud && data.cloud.length > 0) {
      mapCtx.fillStyle = 'rgba(57,255,132,0.6)';
      for (const p of data.cloud) {
        const px = w2c(p.x, p.y);
        mapCtx.fillRect(px.x - 1, px.y - 1, 2, 2);
      }
    }

    if (data.path && data.path.length > 1) {
      mapCtx.strokeStyle = 'rgba(255,170,0,0.7)';
      mapCtx.lineWidth   = 1.5;
      mapCtx.beginPath();
      const p0 = w2c(data.path[0].x, data.path[0].y);
      mapCtx.moveTo(p0.x, p0.y);
      for (let i = 1; i < data.path.length; i++) {
        const pp = w2c(data.path[i].x, data.path[i].y);
        mapCtx.lineTo(pp.x, pp.y);
      }
      mapCtx.stroke();
    }

    if (data.pose) {
      const rp  = w2c(data.pose.x, data.pose.y);
      const yaw = data.pose.yaw;
      const r   = 10;

      mapCtx.save();
      mapCtx.translate(rp.x, rp.y);
      mapCtx.rotate(-yaw);
      mapCtx.strokeStyle = '#00e5ff';
      mapCtx.lineWidth   = 2;
      mapCtx.shadowBlur  = 10;
      mapCtx.shadowColor = '#00e5ff';
      mapCtx.beginPath();
      mapCtx.arc(0, 0, r, 0, Math.PI * 2);
      mapCtx.stroke();
      mapCtx.beginPath();
      mapCtx.moveTo(0, 0);
      mapCtx.lineTo(r + 6, 0);
      mapCtx.stroke();
      mapCtx.restore();

      const deg = (data.pose.yaw * 180 / Math.PI).toFixed(1);
      document.getElementById('map-pose-lbl').textContent =
        `x: ${data.pose.x.toFixed(2)}  y: ${data.pose.y.toFixed(2)}  θ: ${deg}°`;
    }

    const op = w2c(0, 0);
    mapCtx.strokeStyle = '#4a6080';
    mapCtx.lineWidth   = 1;
    mapCtx.beginPath(); mapCtx.moveTo(op.x-8,op.y); mapCtx.lineTo(op.x+8,op.y); mapCtx.stroke();
    mapCtx.beginPath(); mapCtx.moveTo(op.x,op.y-8); mapCtx.lineTo(op.x,op.y+8); mapCtx.stroke();

    const barM  = niceBarLen(5 / mapZoom);
    const barPx = barM * mapZoom;
    mapCtx.strokeStyle = '#4a6080'; mapCtx.lineWidth = 2;
    mapCtx.beginPath(); mapCtx.moveTo(14, H-14); mapCtx.lineTo(14+barPx, H-14); mapCtx.stroke();
    mapCtx.fillStyle = '#4a6080'; mapCtx.font = '9px Share Tech Mono';
    mapCtx.fillText(barM >= 1 ? barM.toFixed(0)+'m' : (barM*100).toFixed(0)+'cm', 14, H-18);
  }

  function niceBarLen(target) {
    const niceVals = [0.1,0.25,0.5,1,2,5,10,20,50];
    return niceVals.reduce((p,x) => Math.abs(x-target)<Math.abs(p-target)?x:p);
  }

  function mapFitView() {
    if (!lastMapData || !lastMapData.path || lastMapData.path.length < 2) return;
    let minX=Infinity,maxX=-Infinity,minY=Infinity,maxY=-Infinity;
    for (const p of lastMapData.path) {
      minX=Math.min(minX,p.x); maxX=Math.max(maxX,p.x);
      minY=Math.min(minY,p.y); maxY=Math.max(maxY,p.y);
    }
    const spanX = maxX-minX+2, spanY = maxY-minY+2;
    mapZoom = Math.min(mapCanvas.width/spanX, mapCanvas.height/spanY) * 0.85;
    mapPanX = -((minX+maxX)/2) * mapZoom;
    mapPanY =  ((minY+maxY)/2) * mapZoom;
    drawMap(lastMapData);
  }

  function mapResetZoom() { mapZoom=40; mapPanX=0; mapPanY=0; drawMap(lastMapData); }

  async function resetMap() {
    await fetch('/map/reset', {method:'POST'});
    lastMapData = null;
    drawMap(null);
  }

  mapCanvas.addEventListener('wheel', e => {
    e.preventDefault();
    mapZoom = Math.max(5, Math.min(200, mapZoom * (e.deltaY < 0 ? 1.15 : 1/1.15)));
    drawMap(lastMapData);
  }, {passive:false});
  mapCanvas.addEventListener('mousedown', e => {
    mapDragging = true;
    mapDragStart = {x:e.clientX, y:e.clientY, px:mapPanX, py:mapPanY};
    mapCanvas.style.cursor = 'grabbing';
  });
  window.addEventListener('mousemove', e => {
    if (!mapDragging) return;
    mapPanX = mapDragStart.px + (e.clientX - mapDragStart.x);
    mapPanY = mapDragStart.py + (e.clientY - mapDragStart.y);
    drawMap(lastMapData);
  });
  window.addEventListener('mouseup', () => { mapDragging=false; mapCanvas.style.cursor='grab'; });
  mapCanvas.addEventListener('dblclick', mapResetZoom);

  let lastMapData = null;
  async function fetchMap() {
    try {
      const d = await (await fetch('/map')).json();
      lastMapData = d;
      drawMap(d);
    } catch(e) {}
    // Map data only matters in auto mode — poll very slowly in manual.
    setTimeout(fetchMap, currentMode === 'manual' ? 5000 : 2000);
  }
  fetchMap();

</script>
</body>
</html>
<!-- " -->
)html";

/**
 * @brief Application entry point.
 *
 * Configures the system by loading `config.txt`, initializing hardware ports,
 * and dispatching all background task threads (lidar, camera, brain). Configures
 * and blocks on the `crow` web app to handle client connections. 
 *
 * @return int Standard exit status (0 on graceful shutdown, non-zero on failure).
 */
int main() {
    signal(SIGINT, signalHandler);
	signal(SIGPIPE, SIG_IGN);
    
    loadConfig("config.txt", CFG);

    
    LidarController lidar(CFG.lidarPort, CFG.flipLidar);
    lidar.setExcludeZones(CFG.lidarExclude);
    if (!lidar.initialize()) {
        std::cerr << "Failed to initialize lidar\n";
        return -1;
    }

    
    CamController cam(0, CFG.camWidth, CFG.camHeight);
    for (const auto& c : CFG.colors)
        cam.addTrackedColor(c.name,
            c.lHueMin, c.lHueMax, c.lSatMin, c.lValMin,
            c.uHueMin, c.uHueMax, c.uSatMin, c.uValMin);
    cam.setFlip(CFG.flipCamera);
    cam.setTrackingStrategy(CamController::TrackingStrategy::LARGEST);
    cam.setMinArea(CFG.minArea);
    cam.setDeadOnThreshold(CFG.deadOnThresh);
    cam.setCollectionZone(CFG.brainCollectXMin, CFG.brainCollectXMax);
    if (!cam.initialize()) {
        std::cerr << "Failed to initialize camera\n";
        return -1;
    }
    cam.enableStreaming(true, CFG.streamPort);
    std::cout << "MJPEG stream : http://<your-pi-ip>:" << CFG.streamPort << "\n";

    
    MotorController motor(CFG.motorPort);
    if (!motor.isOpen()) {
        std::cerr << "Warning: MotorController failed to open " << CFG.motorPort
                  << " — motor commands will be ignored\n";
    } else {
        motor.eStop();
    }
    global_motor_bus.hw = &motor;

    
    std::thread t_lidar(lidarThread,  std::ref(lidar));
    std::thread t_cam  (cameraThread, std::ref(cam));
    std::thread t_brain(brainThread);

    
    crow::SimpleApp app;

    CROW_ROUTE(app, "/")([&](){ return DASHBOARD_HTML; });

    CROW_ROUTE(app, "/data")([&](){
        crow::json::wvalue x;
        std::lock_guard<std::mutex> lk(scan_mutex);
        for (size_t i = 0; i < global_scan.size(); i++) {
            x[i]["a"] = global_scan[i].angle;
            x[i]["r"] = global_scan[i].range;
        }
        return x;
    });

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

    CROW_ROUTE(app, "/motor").methods(crow::HTTPMethod::POST)([&](const crow::request& req){
        {
            std::lock_guard<std::mutex> lk(brain_mutex);
            if (global_brain.enabled)
                return crow::response(409, "brain active — disable autonomous mode first");
        }
        auto body = crow::json::load(req.body);
        if (!body) return crow::response(400, "bad json");

        MotorCommand cmd;
        cmd.cmd_enable = body["cmd"].i()    != 0;
        cmd.direction  = body["direction"].i();
        cmd.turn       = body["turn"].i();
        cmd.speed      = body["speed"].i();
        cmd.pickup     = body["pickup"].i() != 0;
        global_motor_bus.drive(cmd, "manual");
        return crow::response(200, "ok");
    });

    CROW_ROUTE(app, "/motor/estop").methods(crow::HTTPMethod::POST)([&](){
        {
            std::lock_guard<std::mutex> lk(brain_mutex);
            global_brain.enabled = false;
            global_brain.mode    = "IDLE";
        }
        global_motor_bus.eStop("estop-btn");
        return crow::response(200, "ok");
    });

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

    CROW_ROUTE(app, "/map")([&](){
        crow::json::wvalue x;

        auto pose  = global_map.getPose();
        auto path  = global_map.getPath();
        auto cloud = global_map.getCloud();
        auto grid  = global_map.getGrid();

        x["pose"]["x"]   = pose.x;
        x["pose"]["y"]   = pose.y;
        x["pose"]["yaw"] = pose.yaw;

        for (size_t i = 0; i < path.size(); i++) {
            x["path"][i]["x"] = path[i].x;
            x["path"][i]["y"] = path[i].y;
        }

        size_t step = std::max((size_t)1, cloud.size() / 4000);
        size_t ci   = 0;
        for (size_t i = 0; i < cloud.size(); i += step) {
            x["cloud"][ci]["x"] = cloud[i].x();
            x["cloud"][ci]["y"] = cloud[i].y();
            ci++;
        }

        x["grid"]["width"]   = grid.width;
        x["grid"]["height"]  = grid.height;
        x["grid"]["res"]     = grid.res;
        x["grid"]["originX"] = grid.originX;
        x["grid"]["originY"] = grid.originY;

        crow::json::wvalue rle;
        size_t ri = 0;
        size_t gi = 0;
        while (gi < grid.cells.size()) {
            int8_t val   = grid.cells[gi];
            size_t count = 1;
            while (gi + count < grid.cells.size() && grid.cells[gi+count] == val && count < 255)
                count++;
            rle[ri]["v"] = (int)val;
            rle[ri]["n"] = (int)count;
            ri++;
            gi += count;
        }
        x["grid"]["rle"] = std::move(rle);

        return x;
    });

    CROW_ROUTE(app, "/map/reset").methods(crow::HTTPMethod::POST)([&](){
        global_map.reset();
        return crow::response(200, "ok");
    });

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
            MotorCommand stop;
            stop.speed = 0;
            global_motor_bus.drive(stop, "manual");
        } else {
            global_motor_bus.resume();
        }
        std::cout << "[brain] autonomous mode " << (enable ? "ENABLED" : "DISABLED") << "\n";
        return crow::response(200, "ok");
    });

    crow::SimpleApp wsApp;

    CROW_WEBSOCKET_ROUTE(wsApp, "/ws/motor")
        .onopen([&](crow::websocket::connection& conn){
            std::cout << "[WS] motor client connected\n";
            (void)conn;
        })
        .onclose([&](crow::websocket::connection& conn, const std::string&, uint16_t){
            
            std::cout << "[WS] motor client disconnected — stopping\n";
            MotorCommand stop;
            stop.speed = 0;
            global_motor_bus.drive(stop, "ws-disconnect");
            (void)conn;
        })
        .onmessage([&](crow::websocket::connection& /*conn*/,
                        const std::string& data, bool /*is_binary*/){
            
            {
                std::lock_guard<std::mutex> lk(brain_mutex);
                if (global_brain.enabled) return;
            }
            auto body = crow::json::load(data);
            if (!body) return;
            MotorCommand cmd;
            cmd.cmd_enable = body["cmd"].i()    != 0;
            cmd.direction  = body["direction"].i();
            cmd.turn       = body["turn"].i();
            cmd.speed      = body["speed"].i();
            cmd.pickup     = body["pickup"].i() != 0;
            global_motor_bus.drive(cmd, "manual");
        });

    std::cout << "Dashboard    : http://<your-pi-ip>:" << CFG.webPort << "\n";
    std::cout << "Motor WS     : ws://<your-pi-ip>:"   << CFG.wsPort  << "/ws/motor\n";

    wsApp.loglevel(crow::LogLevel::WARNING);
    auto wsFuture = wsApp.port(CFG.wsPort).multithreaded().run_async();

    app.loglevel(crow::LogLevel::WARNING);
    app.port(CFG.webPort).multithreaded().run();

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
