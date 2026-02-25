#ifndef LIDAR_CONTROLLER_H
#define LIDAR_CONTROLLER_H

#include "CYdLidar.h"
#include "YDlidarDriver.h"
#include "ydlidar_protocol.h" // <--- This contains LidarPropModel and YDLIDAR_X4
#include <vector>
#include <string>

struct LidarPoint {
    float angle;
    float range;
};

class LidarController {
public:
    LidarController(const std::string& port);
    ~LidarController();
    bool initialize();
    std::vector<LidarPoint> getLatestScan();

private:
    CYdLidar laser;
    std::string portName;
};

#endif
