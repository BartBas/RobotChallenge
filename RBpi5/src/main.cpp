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
    
    
    const long FRAME_TIME_US = 1000000 / TARGET_FPS;
    
    
    CamController camController(0, 2560, 400);
    globalCamController = &camController;
    
    
    signal(SIGINT, signalHandler);
    
    
    if (!camController.initialize()) {
        cerr << "Error: Cannot initialize camera" << endl;
        return -1;
    }
    
    camController.setFlip(true);
    camController.setTrackingStrategy(CamController::TrackingStrategy::LARGEST);
    camController.setMinArea(500.0);
    camController.setDeadOnThreshold(5.0);
    
    // Enable streaming
    if (ENABLE_STREAMING) {
        camController.enableStreaming(true, STREAM_PORT);
        cout << "Stream available at http://<your-pi-ip>:" << STREAM_PORT << endl;
    }
    
    cout << "Camera ready. Target FPS: " << TARGET_FPS << endl;
    cout << "Press Ctrl+C to exit." << endl;
    
    
    int frameCount = 0;
    auto fpsCounterStart = steady_clock::now();    
    while (true) {        
        auto frameStart = steady_clock::now();                
        if (!camController.captureFrame()) {
            cerr << "Error: Failed to capture frame" << endl;
            break;
        }        
        CamController::Direction dir = camController.getDirection();        

        cout << "Objects detected: " << dir.objectCount 
             << " | Direction: " << dir.command 
             << " | Angle: " << dir.angle << "°" << endl;               
        frameCount++;

        auto now = steady_clock::now();
        auto elapsed = duration_cast<seconds>(now - fpsCounterStart).count();

        if (elapsed >= 1) {
            cout << "Actual FPS: " << frameCount << endl;
            frameCount = 0;
            fpsCounterStart = now;
        }                

        auto frameEnd = steady_clock::now();
        auto frameElapsed = duration_cast<microseconds>(frameEnd - frameStart).count();
        long sleepTime = FRAME_TIME_US - frameElapsed;
        
        if (sleepTime > 0) {
            usleep(sleepTime);
        } else {
            
            
        }
    }    
    camController.release();    
    return 0;
}
