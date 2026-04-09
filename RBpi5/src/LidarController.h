#ifndef LIDAR_CONTROLLER_H
#define LIDAR_CONTROLLER_H

/**
 * @file LidarController.h
 * @brief Interface for the YDLidar sensor used in obstacle detection.
 *
 * @details
 * Wraps the YDLidar SDK into a simple interface that the rest of the
 * codebase can call without touching SDK types directly.
 *
 * ### Responsibilities
 * - Open and configure the serial connection to the lidar.
 * - Start the motor and wait for it to stabilise.
 * - Return a filtered, ready-to-use point cloud each time `getLatestScan()`
 *   is called.
 *
 * ### Filtering Applied to Each Scan
 * | Filter                  | Detail                                                        |
 * |-------------------------|---------------------------------------------------------------|
 * | Range validity          | Points with distance ≤ 0, non-finite, or outside 0.12–10 m are dropped |
 * | Flip correction         | If `flipMounted` is true, all angles are rotated 180°         |
 * | Exclusion zones         | Points whose angle falls within any registered `ExcludeZone` are dropped (used to blank out the robot's own chassis/wheels) |
 *
 * ### Angle Convention
 * Angles are in degrees, 0°–360°, where 0°/360° is the forward direction of
 * the robot (matching the motor direction convention used by `Brain`).
 *
 * ### Usage
 * @code
 * LidarController lidar("/dev/ttyUSB0", false);
 * lidar.setExcludeZones(cfg.lidarExclude);
 * if (lidar.initialize()) {
 *     auto scan = lidar.getLatestScan();
 * }
 * @endcode
 */

#include "CYdLidar.h"
#include "YDlidarDriver.h"
#include "ydlidar_protocol.h"
#include "Config.h"
#include <vector>
#include <string>

struct LidarPoint {
    float angle;
    float range;
};

class LidarController {
public:
    LidarController(const std::string& port, bool flip = false);
    ~LidarController();

    bool initialize();
    std::vector<LidarPoint> getLatestScan();

    void setFlip(bool flip);
    bool isFlipped() const;

    void setExcludeZones(const std::vector<ExcludeZone>& zones);

private:
    std::string              portName;
    bool                     flipMounted;
    CYdLidar                 laser;
    std::vector<ExcludeZone> excludeZones_;

    bool isExcluded(float angle) const;
};

#endif // LIDAR_CONTROLLER_H