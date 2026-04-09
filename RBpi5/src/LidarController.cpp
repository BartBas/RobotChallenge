/**
 * @file LidarController.cpp
 * @brief Implementation of the YDLidar sensor wrapper.
 *
 * @details
 * Uses the YDLidar SDK (`YDlidarDriver`) directly to connect, start, and
 * retrieve scan data from the sensor over a serial port.
 *
 * ### Initialisation Sequence
 * 1. Connect to the serial port at 128 000 baud.
 * 2. Enable single-channel mode.
 * 3. Start the scan motor.
 * 4. Wait 2 seconds for the motor to reach stable speed before the first
 *    scan is usable.
 *
 * ### Scan Retrieval (`getLatestScan`)
 * Calls `grabScanData()` with a 500 ms timeout, then sorts points with
 * `ascendScanData()`. Each raw point is:
 * - Decoded from fixed-point SDK units (angle ÷ 128 → degrees,
 *   distance ÷ 4000 → metres).
 * - Dropped if distance is zero, non-finite, or outside the 0.12–10 m
 *   working range.
 * - Angle-flipped by 180° when `flipMounted` is true.
 * - Dropped if the angle falls within any registered exclusion zone.
 *
 * ### Thread Safety
 * `getLatestScan()` is not internally synchronised. If called from multiple
 * threads, the caller must provide external locking.
 */

#include "LidarController.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include "YDlidarDriver.h"

using namespace ydlidar;

YDlidarDriver drv;

LidarController::LidarController(const std::string& port, bool flip)
    : portName(port)
    , flipMounted(flip)
{}

LidarController::~LidarController() {
    drv.stop();
    drv.disconnect();
}

void LidarController::setFlip(bool flip) {
    flipMounted = flip;
}

bool LidarController::isFlipped() const {
    return flipMounted;
}

bool LidarController::initialize() {
    result_t op_result = drv.connect(portName.c_str(), 128000);
    if (!IS_OK(op_result)) return false;

    drv.setSingleChannel(true);
    op_result = drv.startScan();

    if (IS_OK(op_result)) {
        std::cout << "Lidar: Motor stabilizing..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        return true;
    }
    return false;
}

void LidarController::setExcludeZones(const std::vector<ExcludeZone>& zones) {
    excludeZones_ = zones;
}

bool LidarController::isExcluded(float angle) const {
    for (const auto& z : excludeZones_)
        if (angle >= z.angleMin && angle <= z.angleMax)
            return true;
    return false;
}

std::vector<LidarPoint> LidarController::getLatestScan() {
    std::vector<LidarPoint> points;
    node_info nodes[1000];
    size_t count = 1000;

    result_t op_result = drv.grabScanData(nodes, count, 500);

    if (IS_OK(op_result) && count > 0) {
        drv.ascendScanData(nodes, count);

        for (size_t i = 0; i < count; i++) {
            float angle    = (float)nodes[i].angle / 128.0f;
            float distance = (float)nodes[i].dist  / 4000.0f;

            if (distance <= 0.0f || !std::isfinite(distance) || !std::isfinite(angle)) continue;
            if (distance > 0.12f && distance < 10.0f) {
                if (flipMounted)
                    angle = std::fmod(angle + 180.0f, 360.0f);
                if (!isExcluded(angle))
                    points.push_back({angle, distance});
            }
        }
    }

    return points;
}