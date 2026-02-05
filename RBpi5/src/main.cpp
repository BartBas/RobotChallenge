#include "CamController.h"
#include <iostream>
#include <csignal>

using namespace std;

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
    // Create controller
    CamController camController(0, 640, 480);
    globalCamController = &camController;
    
    // Setup signal handler for clean shutdown
    signal(SIGINT, signalHandler);
    
    // Initialize camera
    if (!camController.initialize()) {
        cerr << "Error: Cannot initialize camera" << endl;
        return -1;
    }
    
    // Set tracking strategy
    camController.setTrackingStrategy(CamController::TrackingStrategy::LARGEST);
    
    // Optional: Adjust parameters
    camController.setMinArea(500.0);
    camController.setDeadOnThreshold(5.0);
    
    // Enable streaming (true/false, port)
    bool enableStream = true;  // SET THIS TO false TO DISABLE
    if (enableStream) {
        camController.enableStreaming(true, 8080);
        cout << "Stream available at http://<your-pi-ip>:8080" << endl;
    }
    
    cout << "Camera ready. Press Ctrl+C to exit." << endl;
    
    while (true) {
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
        
        // Small delay
        usleep(33000); // ~30 FPS
    }
    
    camController.release();
    
    return 0;
}
