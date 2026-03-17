#ifndef LIDAR_CONTROLLER_H
#define LIDAR_CONTROLLER_H

#include "CYdLidar.h"
#include "YDlidarDriver.h"
#include "ydlidar_protocol.h"
#include <vector>
#include <string>

struct LidarPoint {
    float angle;
    float range;
};

class LidarController {
public:
    // flip = true when the lidar is physically mounted upside-down
    LidarController(const std::string& port, bool flip = false);
    ~LidarController();

    bool initialize();
    std::vector<LidarPoint> getLatestScan();

    void setFlip(bool flip);
    bool isFlipped() const;

private:
    std::string portName;
    bool        flipMounted;
    CYdLidar    laser;
};

#endif
