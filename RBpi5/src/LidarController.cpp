#include "LidarController.h"
#include <iostream>
#include <thread>
#include <chrono>
#include "YDlidarDriver.h"

using namespace ydlidar;

// Global driver instance
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
        // Reduced sleep to get data flowing sooner
        std::this_thread::sleep_for(std::chrono::seconds(2));
        return true;
    }
    return false;
}

std::vector<LidarPoint> LidarController::getLatestScan() {
    std::vector<LidarPoint> points;
    node_info nodes[1000]; 
    size_t count = 1000;

    // Grab data with a shorter timeout to keep the loop moving
    result_t op_result = drv.grabScanData(nodes, count, 500);
    
    if (IS_OK(op_result) && count > 0) {
        for (size_t i = 0; i < count; i++) {
            float distance = (float)nodes[i].dist / 1000.0f; 
            float angle = nodes[i].angle;

            // X4PRO Filter
            if (distance > 0.12f && distance < 10.0f) {
                // We use LidarPoint{angle, distance} 
                // Ensure these match the struct in your .h file!
                points.push_back({angle, distance});
            }
        }
    } else {
        // If it fails, print a debug message so you know WHY the front end is empty
        // std::cerr << "LiDAR: Failed to grab data or timed out." << std::endl;
    }
    
    return points;
}
