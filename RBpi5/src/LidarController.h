#ifndef LIDAR_CONTROLLER_H
#define LIDAR_CONTROLLER_H

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

    // Wheel / chassis exclusion zones — points in these angle ranges are dropped
    void setExcludeZones(const std::vector<ExcludeZone>& zones);

private:
    std::string              portName;
    bool                     flipMounted;
    CYdLidar                 laser;
    std::vector<ExcludeZone> excludeZones_;

    bool isExcluded(float angle) const;
};

#endif
