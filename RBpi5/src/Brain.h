#ifndef BRAIN_H
#define BRAIN_H

// ══════════════════════════════════════════════════════════════════════════════
//  Brain.h  —  Autonomous decision-making for the cup-collecting robot
//
//  Include this header in main.cpp (or any translation unit) and call
//  Brain::tick() from your brain thread.
//
//  State machine:
//
//    IDLE     — brain disabled, do nothing
//    SEEK     — no target visible, spin slowly to find one
//    CHASE    — target visible and far away, steer toward it smoothly
//               using the full 1–360° heading range (e.g. 350° = slight left,
//               10° = slight right, 360° = straight forward)
//    COLLECT  — target is close; three sub-phases:
//                 Phase A (SIDESTEP)   — strafe left until cup centre-x falls
//                                        inside the collection window
//                 Phase B (DRIVE_OVER) — drive straight forward for a fixed
//                                        time (brain_drive_over_ms ms); camera
//                                        will lose the cup during this phase,
//                                        so we ignore tracking and just commit
//                 Phase C (DONE)       — stop and reset so brain can seek again
//    AVOID    — obstacle too close in the front arc, turn away
//
//  Priority (highest wins at each tick):
//    1. E-stop active                  → IDLE   (hold still)
//    2. COLLECT phase B active         → finish the timed drive (camera-blind)
//    3. Obstacle < clearDist           → AVOID  (turn away)
//    4. Target visible & close         → COLLECT phase A (sidestep)
//    5. Target visible & far           → CHASE
//    6. No target                      → SEEK
//
//  Note: AVOID does NOT interrupt an in-progress Phase B drive-over.
//        If you want it to, swap priorities 2 and 3 below.
//
//  Direction convention (YOUR motor firmware):
//    360 (or 0) = forward
//     90        = right
//    180        = backward
//    270        = left
//    Note: sending 0 does NOT move the robot (firmware dead-zone),
//          so forward is always sent as 360.
//
//  Configuration keys (all live in RobotConfig / config.txt):
//    brain_collect_dist      — pixel area threshold above which COLLECT activates
//    brain_collect_x_min     — left  pixel boundary of collection window (0–1 normalised)
//    brain_collect_x_max     — right pixel boundary of collection window (0–1 normalised)
//    brain_sidestep_speed    — speed used while side-stepping left during COLLECT
//    brain_drive_speed       — speed used while driving over the cup in COLLECT
//    brain_drive_over_ms     — how long (ms) to drive forward blindly in Phase B
//                              e.g. 1500 = 1.5 seconds. Tune by measuring how
//                              far the robot travels at brain_drive_speed.
//
// ══════════════════════════════════════════════════════════════════════════════

#include "Config.h"
#include "LidarController.h"
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <iostream>

// ── MotorCommand (duplicated here so Brain.h is self-contained) ──────────────
//    If you already have this struct in main.cpp, guard with a macro.
#ifndef MOTOR_COMMAND_DEFINED
#define MOTOR_COMMAND_DEFINED
struct MotorCommand {
    bool   cmd_enable = true;
    int    direction  = 360;  // compass degrees 1-360  (360 = straight forward)
    int    turn       = 0;    // 0=none  1=left  2=right
    int    speed      = 0;    // 0-100 %
    bool   pickup     = false;
};
#endif

// ── TrackingSnapshot ─────────────────────────────────────────────────────────
//    A plain-data snapshot of whatever the camera thread computed.
//    Fill this from your global_tracking struct before calling Brain::tick().
struct TrackingSnapshot {
    double      angle        = 0.0;   // bearing: neg=left, pos=right  (-90..+90°)
    double      distance     = 0.0;   // not used directly, kept for future use
    int         objectCount  = 0;
    std::string command      = "NO TARGET";

    // Pixel-space info (needed for COLLECT logic)
    double      targetPixelX = -1.0;  // centre-x in pixels (-1 = no object)
    double      targetPixelY = -1.0;
    double      targetArea   = 0.0;   // contour area in pixels²
    int         frameWidth   = 640;   // current frame width  (for normalisation)
};

// ── BrainOutput ──────────────────────────────────────────────────────────────
//    Everything the brain wants to say after one tick.
struct BrainOutput {
    MotorCommand cmd;
    std::string  mode        = "IDLE";
    std::string  reason;
    float        frontClear  = -1.f;  // nearest obstacle in front arc (m), -1 = clear
};

// ════════════════════════════════════════════════════════════════════════════
//  Brain class
// ════════════════════════════════════════════════════════════════════════════
class Brain {
public:
    // ── Construction ─────────────────────────────────────────────────────────
    explicit Brain(const RobotConfig& cfg)
        : cfg_(cfg)
    {}

    // ── tick ─────────────────────────────────────────────────────────────────
    //  Call once per brain period.
    //  estopped   : true if the MotorBus e-stop flag is set
    //  scan       : latest lidar scan (may be empty while lidar warms up)
    //  tracking   : latest camera snapshot
    //  Returns a BrainOutput describing what to do next.
    BrainOutput tick(bool                          estopped,
                     const std::vector<LidarPoint>& scan,
                     const TrackingSnapshot&        tracking)
    {
        BrainOutput out;

        // ── 1. Obstacle detection — front arc ─────────────────────────────
        // Lidar 0°/360° is forward in your convention.
        float frontClosest = 9999.f;
        float worstAngle   = 0.f;

        for (const auto& p : scan) {
            // Normalise to -180..+180 relative to forward (0°/360°)
            float rel = p.angle;
            while (rel >  180.f) rel -= 360.f;
            while (rel < -180.f) rel += 360.f;

            if (std::abs(rel) < cfg_.brainFrontArc && p.range < frontClosest) {
                frontClosest = p.range;
                worstAngle   = rel;
            }
        }
        out.frontClear = (frontClosest > 99.f) ? -1.f : frontClosest;

        // ── 2. Decision tree ──────────────────────────────────────────────

        if (estopped) {
            // ── IDLE / E-stop ─────────────────────────────────────────────
            out.mode           = "IDLE";
            out.reason         = "e-stop active";
            out.cmd.speed      = 0;
            out.cmd.cmd_enable = false;
            // Reset any in-progress collection so we start clean after e-stop
            collectingPhaseB_ = false;

        } else if (collectingPhaseB_) {
            // ── COLLECT Phase B — timed blind drive ───────────────────────
            // We are committed to driving over the cup. The camera has lost
            // it by now; we just run the clock out.
            // AVOID is intentionally skipped here — if you want obstacle
            // avoidance to interrupt the drive-over, move this block below
            // the frontClosest check.
            out = driveOverTick();

        } else if (frontClosest < cfg_.brainClearDist) {
            // ── AVOID ─────────────────────────────────────────────────────
            out.mode          = "AVOID";
            out.reason        = "obstacle at " + std::to_string((int)(frontClosest * 100)) + " cm";
            out.cmd.speed     = cfg_.brainAvoidSpeed;
            out.cmd.direction = 360;
            out.cmd.turn      = (worstAngle > 0) ? 2 : 1;   // 2=right  1=left

        } else if (tracking.objectCount > 0 &&
                   tracking.targetArea >= cfg_.brainCollectDist) {
            // ── COLLECT Phase A — sidestep to align ───────────────────────
            out = collectSidestepTick(tracking);

        } else if (tracking.objectCount > 0) {
            // ── CHASE — steer toward bearing using full heading range ──────
            //
            //  Camera bearing:  -90° (hard left) … 0° (dead ahead) … +90° (hard right)
            //  Motor heading:   360° = forward,  270° = left,  90° = right
            //
            //  Mapping:  heading = 360 + camera_angle
            //    angle =   0  →  360  (straight forward)
            //    angle = -30  →  330  (gentle left diagonal)
            //    angle = +30  →  390 → wrapped to 30  (gentle right diagonal)
            //    angle = -90  →  270  (pure left strafe)
            //    angle = +90  →  450 → wrapped to 90  (pure right strafe)
            //
            int heading = 360 + static_cast<int>(std::round(tracking.angle));
            heading     = ((heading - 1) % 360) + 1;   // keep in 1..360

            out.mode          = "CHASE";
            out.reason        = "target at " + std::to_string((int)tracking.angle) +
                                " deg → heading " + std::to_string(heading);
            out.cmd.speed     = cfg_.brainChaseSpeed;
            out.cmd.direction = heading;
            out.cmd.turn      = 0;

        } else {
            // ── SEEK — spin to find target ────────────────────────────────
            out.mode          = "SEEK";
            out.reason        = "no target visible";
            out.cmd.speed     = cfg_.brainSeekSpeed;
            out.cmd.direction = 360;
            out.cmd.turn      = 1;   // spin left
        }

        return out;
    }

private:
    const RobotConfig& cfg_;

    using Clock     = std::chrono::steady_clock;
    using TimePoint = std::chrono::time_point<Clock>;

    bool      collectingPhaseB_ = false;
    TimePoint driveOverStart_;

    // ── collectSidestepTick (Phase A) ─────────────────────────────────────────
    //  Strafe left until the cup centre-x enters the collection window, then
    //  kick off the timed drive-over (Phase B).
    BrainOutput collectSidestepTick(const TrackingSnapshot& tracking)
    {
        BrainOutput out;

        // Normalise target X to 0..1
        double normX = (tracking.frameWidth > 0 && tracking.targetPixelX >= 0)
                       ? (tracking.targetPixelX / tracking.frameWidth)
                       : 0.5;

        bool inWindow = (normX >= cfg_.brainCollectXMin &&
                         normX <= cfg_.brainCollectXMax);

        if (!inWindow) {
            // Still sidestepping
            out.mode      = "COLLECT";
            out.reason    = "sidestepping left — cup at " +
                            std::to_string((int)(normX * 100)) + "% (window: " +
                            std::to_string((int)(cfg_.brainCollectXMin * 100)) + "–" +
                            std::to_string((int)(cfg_.brainCollectXMax * 100)) + "%)";
            out.cmd.speed     = cfg_.brainSidestepSpeed;
            out.cmd.direction = 270;   // pure left strafe
            out.cmd.turn      = 0;
            out.cmd.pickup    = false;
        } else {
            // Cup is aligned — start the timed blind drive
            collectingPhaseB_ = true;
            driveOverStart_   = Clock::now();
            out = driveOverTick();   // begin immediately this tick
        }

        return out;
    }

    // ── driveOverTick (Phase B) ───────────────────────────────────────────────
    //  Drive forward blindly for brain_drive_over_ms milliseconds.
    //  The camera cannot see the cup at this point — we just commit to the
    //  motion and trust the timing.
    //  Once the timer expires, stop and clear the flag so the brain returns
    //  to SEEK for the next cup.
    BrainOutput driveOverTick()
    {
        BrainOutput out;

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           Clock::now() - driveOverStart_).count();

        if (elapsed < static_cast<long long>(cfg_.brainDriveOverMs)) {
            // Still within the drive-over window
            out.mode      = "COLLECT";
            out.reason    = "driving over cup — " +
                            std::to_string(elapsed) + " / " +
                            std::to_string(cfg_.brainDriveOverMs) + " ms";
            out.cmd.speed     = cfg_.brainDriveSpeed;
            out.cmd.direction = 360;   // straight forward
            out.cmd.turn      = 0;
            out.cmd.pickup    = true;  // keep pickup servo running throughout
        } else {
            // Timer expired — cup should be scooped
            collectingPhaseB_ = false;
            out.mode      = "COLLECT";
            out.reason    = "drive-over complete — resuming search";
            out.cmd.speed     = 0;
            out.cmd.direction = 360;
            out.cmd.turn      = 0;
            out.cmd.pickup    = false;
        }

        return out;
    }
};

#endif // BRAIN_H
