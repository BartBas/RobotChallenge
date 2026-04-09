#ifndef BRAIN_H
#define BRAIN_H

/**
 * @file Brain.h
 * @brief Autonomous decision-making module for the cup-collecting robot.
 *
 * @details
 * Implements a priority-based state machine that drives the robot through five
 * operating modes: IDLE, SEEK, CHASE, COLLECT, and AVOID.
 *
 * ### State Machine Overview
 * | State   | Condition                                      | Behaviour                                      |
 * |---------|------------------------------------------------|------------------------------------------------|
 * | IDLE    | E-stop active                                  | Halt all motion                                |
 * | SEEK    | No target visible                              | Spin slowly to scan for a target               |
 * | CHASE   | Target visible, area below collect threshold   | Steer toward target using full heading range   |
 * | COLLECT | Target visible, area above collect threshold   | Three-phase pickup sequence (see below)        |
 * | AVOID   | Obstacle within clearance distance             | Reverse and turn away from obstacle            |
 *
 * ### COLLECT Sub-phases
 * - **Phase A – SIDESTEP**: Strafe left until the cup's centre-x enters the
 *   collection window defined by `brain_collect_x_min` / `brain_collect_x_max`.
 * - **Phase B – DRIVE_OVER**: Drive straight forward for `brain_drive_over_ms`
 *   milliseconds. The camera loses the cup during this phase; motion is
 *   committed on timing alone.
 * - **Phase C – DONE**: Stop and reset so the brain returns to SEEK for the
 *   next cup.
 *
 * ### Priority Order (highest first)
 * 1. E-stop active → IDLE
 * 2. COLLECT Phase B in progress → finish timed drive (camera-blind, AVOID skipped)
 * 3. Obstacle closer than `brainClearDist` → AVOID
 * 4. Target visible and area ≥ `brainCollectDist` → COLLECT Phase A
 * 5. Target visible and area < `brainCollectDist` → CHASE
 * 6. No target → SEEK
 *
 * ### Motor Direction Convention
 * | Value | Direction     |
 * |-------|---------------|
 * | 360   | Forward       |
 * | 90    | Right         |
 * | 180   | Backward      |
 * | 270   | Left (strafe) |
 *
 * @note Sending direction 0 triggers a firmware dead-zone; always use 360 for
 *       straight forward.
 *
 * ### Configuration Keys (config.txt / RobotConfig)
 * | Key                      | Description                                                  |
 * |--------------------------|--------------------------------------------------------------|
 * | brain_collect_dist       | Pixel-area threshold above which COLLECT activates           |
 * | brain_collect_x_min      | Left boundary of collection window (0–1 normalised)          |
 * | brain_collect_x_max      | Right boundary of collection window (0–1 normalised)         |
 * | brain_sidestep_speed     | Speed (0–100 %) used during Phase A side-step                |
 * | brain_drive_speed        | Speed (0–100 %) used during Phase B blind drive              |
 * | brain_drive_over_ms      | Duration (ms) of the Phase B forward drive                   |
 *
 * ### Usage
 * @code
 * Brain brain(config);
 * BrainOutput out = brain.tick(estopped, lidarScan, trackingSnapshot);
 * applyMotorCommand(out.cmd);
 * @endcode
 */

#include "Config.h"
#include "LidarController.h"
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <iostream>

#ifndef MOTOR_COMMAND_DEFINED
#define MOTOR_COMMAND_DEFINED
struct MotorCommand {
    bool   cmd_enable = true;
    int    direction  = 360;
    int    turn       = 0;
    int    speed      = 0;
    bool   pickup     = true;
};
#endif

struct TrackingSnapshot {
    double      angle        = 0.0;
    double      distance     = 0.0;
    int         objectCount  = 0;
    std::string command      = "NO TARGET";

    double      targetPixelX = -1.0;
    double      targetPixelY = -1.0;
    double      targetArea   = 0.0;
    int         frameWidth   = 640;
};

struct BrainOutput {
    MotorCommand cmd;
    std::string  mode        = "IDLE";
    std::string  reason;
    float        frontClear  = -1.f;
};

class Brain {
public:
    explicit Brain(const RobotConfig& cfg)
        : cfg_(cfg)
    {}

    BrainOutput tick(bool                          estopped,
                     const std::vector<LidarPoint>& scan,
                     const TrackingSnapshot&        tracking)
    {
        BrainOutput out;

        float frontClosest = 9999.f;
        float worstAngle   = 0.f;

        for (const auto& p : scan) {
            float rel = p.angle;
            while (rel >  180.f) rel -= 360.f;
            while (rel < -180.f) rel += 360.f;

            if (std::abs(rel) < cfg_.brainFrontArc && p.range < frontClosest) {
                frontClosest = p.range;
                worstAngle   = rel;
            }
        }
        out.frontClear = (frontClosest > 99.f) ? -1.f : frontClosest;

        if (estopped) {
            out.mode           = "IDLE";
            out.reason         = "e-stop active";
            out.cmd.speed      = 0;
            out.cmd.cmd_enable = false;
            collectingPhaseB_ = false;

        } else if (collectingPhaseB_) {
            out = driveOverTick();

        } else if (frontClosest < cfg_.brainClearDist && false) {
            out.mode          = "AVOID";
            out.reason        = "obstacle at " + std::to_string((int)(frontClosest * 100)) + " cm";
            out.cmd.speed     = cfg_.brainAvoidSpeed;
            out.cmd.direction = 180;
            out.cmd.turn      = (worstAngle > 0) ? 2 : 1;

        } else if (tracking.objectCount > 0 &&
                   tracking.targetArea >= cfg_.brainCollectDist) {
            out = collectSidestepTick(tracking);

        } else if (tracking.objectCount > 0) {
            int heading = 360 + static_cast<int>(std::round(tracking.angle));
            heading     = ((heading - 1) % 360) + 1;

            out.mode          = "CHASE";
            out.reason        = "target at " + std::to_string((int)tracking.angle) +
                                " deg → heading " + std::to_string(heading);
            out.cmd.speed     = cfg_.brainChaseSpeed;
            out.cmd.direction = heading;
            out.cmd.turn      = 0;

        } else {
            out.mode          = "SEEK";
            out.reason        = "no target visible";
            out.cmd.speed     = cfg_.brainSeekSpeed;
            out.cmd.direction = 360;
            out.cmd.turn      = 1;
        }

        return out;
    }

private:
    const RobotConfig& cfg_;

    using Clock     = std::chrono::steady_clock;
    using TimePoint = std::chrono::time_point<Clock>;

    bool      collectingPhaseB_ = false;
    TimePoint driveOverStart_;

    BrainOutput collectSidestepTick(const TrackingSnapshot& tracking)
    {
        BrainOutput out;

        double normX = (tracking.frameWidth > 0 && tracking.targetPixelX >= 0)
                       ? (tracking.targetPixelX / tracking.frameWidth)
                       : 0.5;

        bool inWindow = (normX >= cfg_.brainCollectXMin &&
                         normX <= cfg_.brainCollectXMax);

        if (!inWindow) {
            out.mode      = "COLLECT";
            out.reason    = "sidestepping left — cup at " +
                            std::to_string((int)(normX * 100)) + "% (window: " +
                            std::to_string((int)(cfg_.brainCollectXMin * 100)) + "–" +
                            std::to_string((int)(cfg_.brainCollectXMax * 100)) + "%)";
            out.cmd.speed     = cfg_.brainSidestepSpeed;
            out.cmd.direction = 270;
            out.cmd.turn      = 0;
            out.cmd.pickup    = true;
        } else {
            collectingPhaseB_ = true;
            driveOverStart_   = Clock::now();
            out = driveOverTick();
        }

        return out;
    }

    BrainOutput driveOverTick()
    {
        BrainOutput out;

        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                           Clock::now() - driveOverStart_).count();

        if (elapsed < static_cast<long long>(cfg_.brainDriveOverMs)) {
            out.mode      = "COLLECT";
            out.reason    = "driving over cup — " +
                            std::to_string(elapsed) + " / " +
                            std::to_string(cfg_.brainDriveOverMs) + " ms";
            out.cmd.speed     = cfg_.brainDriveSpeed;
            out.cmd.direction = 359;
            out.cmd.turn      = 0;
            out.cmd.pickup    = true;
        } else {
            collectingPhaseB_ = false;
            out.mode      = "COLLECT";
            out.reason    = "drive-over complete — resuming search";
            out.cmd.speed     = 0;
			
            out.cmd.direction = 359; 
            out.cmd.turn      = 0;
            out.cmd.pickup    = false;
        }

        return out;
    }
};

#endif // BRAIN_H