#include "LidarController.h"
#include <iostream>
#include <thread>
#include <chrono>
#include "YDlidarDriver.h"

using namespace ydlidar;

YDlidarDriver drv;

LidarController::LidarController(const std::string& port) : portName(port) {}

LidarController::~LidarController() {
    drv.stop();
    drv.disconnect();
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

std::vector<LidarPoint> LidarController::getLatestScan() {
    std::vector<LidarPoint> points;
    node_info nodes[1000]; 
    size_t count = 1000;

    result_t op_result = drv.grabScanData(nodes, count, 500);
    
    if (IS_OK(op_result) && count > 0) {
        // This is the critical call - it sorts nodes and converts angles
        // to proper 0-360 degree values with a fixed 0° reference point
        drv.ascendScanData(nodes, count);

        for (size_t i = 0; i < count; i++) {
            // Now divide by 64 to get actual degrees (YDLidar fixed-point format)
            float angle = (float)nodes[i].angle / 128.0f;
            float distance = (float)nodes[i].dist / 1000.0f; // mm to meters

            if (distance > 0.12f && distance < 10.0f) {
                points.push_back({angle, distance});
            }
        }
    }
    
    return points;
}
