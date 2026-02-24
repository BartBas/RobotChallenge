#include "CamController.h"
#include <iostream>
#include <csignal>
#include <chrono>

using namespace std;
using namespace std::chrono;

// Global pointer for signal handler
CamController* globalCamController = nullptr;

void signalHandler(int signum) {
    cout << "\nShutting down..." << endl;
    if (globalCamController) {
        globalCamController->release();
    }
    exit(signum);
}

int main() {
    // ===== CONFIGURATION =====
    const int TARGET_FPS = 60;  // Set your desired FPS here
    const bool ENABLE_STREAMING = true;
    const int STREAM_PORT = 8080;
    // =========================
    
    // Calculate frame time in microseconds
    const long FRAME_TIME_US = 1000000 / TARGET_FPS;
    
    // Create controller
    CamController camController(0, 2560, 400);
    globalCamController = &camController;
    
    // Setup signal handler for clean shutdown
    signal(SIGINT, signalHandler);
    
    // Initialize camera
    if (!camController.initialize()) {
        cerr << "Error: Cannot initialize camera" << endl;
        return -1;
    }
    
    // Enable 180° flip (if camera is upside down)
    camController.setFlip(true);
    
    // Set tracking strategy
    camController.setTrackingStrategy(CamController::TrackingStrategy::LARGEST);
    
    // Optional: Adjust parameters
    camController.setMinArea(500.0);
    camController.setDeadOnThreshold(5.0);
    
    // Enable streaming
    if (ENABLE_STREAMING) {
        camController.enableStreaming(true, STREAM_PORT);
        cout << "Stream available at http://<your-pi-ip>:" << STREAM_PORT << endl;
    }
    
    cout << "Camera ready. Target FPS: " << TARGET_FPS << endl;
    cout << "Press Ctrl+C to exit." << endl;
    
    // FPS tracking
    int frameCount = 0;
    auto fpsCounterStart = steady_clock::now();
    
    while (true) {
        // Start frame timer
        auto frameStart = steady_clock::now();
        
        // Capture and process frame
        if (!camController.captureFrame()) {
            cerr << "Error: Failed to capture frame" << endl;
            break;
        }
        
        // Get direction
        CamController::Direction dir = camController.getDirection();
        
        cout << "Objects detected: " << dir.objectCount 
             << " | Direction: " << dir.command 
             << " | Angle: " << dir.angle << "°" << endl;
        
        // Calculate actual FPS every second
        frameCount++;
        auto now = steady_clock::now();
        auto elapsed = duration_cast<seconds>(now - fpsCounterStart).count();
        if (elapsed >= 1) {
            cout << "Actual FPS: " << frameCount << endl;
            frameCount = 0;
            fpsCounterStart = now;
        }
        
        // Smart frame rate control - wait for remaining frame time
        auto frameEnd = steady_clock::now();
        auto frameElapsed = duration_cast<microseconds>(frameEnd - frameStart).count();
        long sleepTime = FRAME_TIME_US - frameElapsed;
        
        if (sleepTime > 0) {
            usleep(sleepTime);
        } else {
            // Frame took longer than target - we're running slower than target FPS
            // (This is fine, just means we can't achieve target FPS)
        }
    }
    
    camController.release();
    
    return 0;
}
