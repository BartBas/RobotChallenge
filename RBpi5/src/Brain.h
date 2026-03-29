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
//    CHASE    — target visible and far away, steer toward it
//    COLLECT  — target is close; shift LEFT until its pixel-centre falls
//               inside the configured collection window, then drive straight
//               forward to scoop it up
//    AVOID    — obstacle too close in the front arc, turn away
//
//  Priority (highest wins at each tick):
//    1. E-stop active           → IDLE   (hold still)
//    2. Obstacle < clearDist    → AVOID  (turn away)
//    3. Target visible & close  → COLLECT
//    4. Target visible & far    → CHASE
//    5. No target               → SEEK
//
//  Configuration keys (all live in RobotConfig / config.txt):
//    brain_collect_dist   — pixel area threshold above which COLLECT activates
//    brain_collect_x_min  — left  pixel boundary of collection window (0–1 normalised)
//    brain_collect_x_max  — right pixel boundary of collection window (0–1 normalised)
//    brain_sidestep_speed — speed used while side-stepping left during COLLECT
//    brain_drive_speed    — speed used while driving over the cup in COLLECT
//
// ══════════════════════════════════════════════════════════════════════════════

#include "Config.h"
#include "LidarController.h"
#include <string>
#include <vector>
#include <cmath>
#include <iostream>

// ── MotorCommand (duplicated here so Brain.h is self-contained) ──────────────
//    If you already have this struct in main.cpp, guard with a macro.
#ifndef MOTOR_COMMAND_DEFINED
#define MOTOR_COMMAND_DEFINED
struct MotorCommand {
    bool   cmd_enable = true;
    int    direction  = 90;   // compass degrees  0-359  (90 = straight forward)
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
        float frontClosest = 9999.f;
        float worstAngle   = 0.f;

        for (const auto& p : scan) {
            float rel = p.angle - 90.f;
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
            out.mode          = "IDLE";
            out.reason        = "e-stop active";
            out.cmd.speed     = 0;
            out.cmd.cmd_enable = false;

        } else if (frontClosest < cfg_.brainClearDist) {
            // ── AVOID ─────────────────────────────────────────────────────
            out.mode      = "AVOID";
            out.reason    = "obstacle at " + std::to_string((int)(frontClosest * 100)) + " cm";
            out.cmd.speed     = cfg_.brainAvoidSpeed;
            out.cmd.direction = 90;
            // Turn away from the side the obstacle is on
            out.cmd.turn  = (worstAngle > 0) ? 2 : 1;   // 2=right away, 1=left away

        } else if (tracking.objectCount > 0 &&
                   tracking.targetArea >= cfg_.brainCollectDist) {
            // ── COLLECT — cup is close enough to pick up ──────────────────
            out = collectTick(tracking);

        } else if (tracking.objectCount > 0) {
            // ── CHASE — steer toward bearing ──────────────────────────────
            out.mode      = "CHASE";
            out.reason    = "target at " + std::to_string((int)tracking.angle) + " deg";
            out.cmd.speed = cfg_.brainChaseSpeed;
            // Map camera bearing (-90..+90) → drive heading (0..359)
            int heading   = 90 + static_cast<int>(tracking.angle);
            heading       = ((heading % 360) + 360) % 360;
            out.cmd.direction = heading;
            out.cmd.turn      = 0;

        } else {
            // ── SEEK — spin to find target ────────────────────────────────
            out.mode          = "SEEK";
            out.reason        = "no target visible";
            out.cmd.speed     = cfg_.brainSeekSpeed;
            out.cmd.direction = 90;
            out.cmd.turn      = 1;   // spin left
        }

        return out;
    }

private:
    const RobotConfig& cfg_;

    // ── collectTick ───────────────────────────────────────────────────────────
    //
    //  Two sub-phases:
    //    Phase A  (SIDESTEP) — slide left until the cup centre-x is inside the
    //                          collection window  [collectXMin .. collectXMax]
    //                          (both expressed as 0..1 fractions of frame width)
    //    Phase B  (DRIVE_OVER) — once aligned, drive straight forward to scoop
    //
    BrainOutput collectTick(const TrackingSnapshot& tracking)
    {
        BrainOutput out;

        // Normalise target X to 0..1
        double normX = (tracking.frameWidth > 0 && tracking.targetPixelX >= 0)
                       ? (tracking.targetPixelX / tracking.frameWidth)
                       : 0.5;

        bool inWindow = (normX >= cfg_.brainCollectXMin &&
                         normX <= cfg_.brainCollectXMax);

        if (!inWindow) {
            // ── Phase A: sidestep left ────────────────────────────────────
            out.mode      = "COLLECT";
            out.reason    = "sidestepping left — cup at " +
                            std::to_string((int)(normX * 100)) + "% (window: " +
                            std::to_string((int)(cfg_.brainCollectXMin * 100)) + "–" +
                            std::to_string((int)(cfg_.brainCollectXMax * 100)) + "%)";
            // Strafe left: direction 180° = pure left in our compass scheme
            out.cmd.speed     = cfg_.brainSidestepSpeed;
            out.cmd.direction = 180;   // left strafe
            out.cmd.turn      = 0;
            out.cmd.pickup    = false;
        } else {
            // ── Phase B: drive over the cup ───────────────────────────────
            out.mode      = "COLLECT";
            out.reason    = "aligned — driving over cup";
            out.cmd.speed     = cfg_.brainDriveSpeed;
            out.cmd.direction = 90;    // straight forward
            out.cmd.turn      = 0;
            out.cmd.pickup    = true;  // activate pickup servo
        }

        return out;
    }
};

#endif // BRAIN_H
