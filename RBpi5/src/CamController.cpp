#include "CamController.h"
#include <algorithm>
#include <cmath>
#include <iostream>

using namespace cv;
using namespace std;

CamController::CamController(int cameraIndex, int width, int height)
    : cameraIndex(cameraIndex)
    , frameWidth(width)
    , frameHeight(height)
    , minArea(500.0)
    , deadOnThreshold(5.0)
    , strategy(TrackingStrategy::LARGEST)
    , cameraStream(nullptr)
    , useRpiCam(false)
    , streamingEnabled(false)
    , streamPort(8080)
    , flip180(false)
{
    redLowerMin = cv::Scalar(0, 100, 100);
    redLowerMax = cv::Scalar(10, 255, 255);
    redUpperMin = cv::Scalar(170, 100, 100);
    redUpperMax = cv::Scalar(180, 255, 255);
}

CamController::~CamController() {
    enableStreaming(false);  // Stop streaming thread
    release();
}

bool CamController::initialize() {
    // Force rpicam-vid for Raspberry Pi Camera Module 3
    cout << "Initializing camera using rpicam-vid..." << endl;
    
    string cmd = "rpicam-vid --width " + to_string(frameWidth) + 
                 " --height " + to_string(frameHeight) + 
                 " --codec yuv420 --framerate 30 -t 0 -o - 2>/dev/null";
    
    cameraStream = popen(cmd.c_str(), "r");
    
    if (cameraStream == nullptr) {
        cerr << "Failed to start rpicam-vid" << endl;
        return false;
    }
    
    useRpiCam = true;
    cout << "Camera initialized successfully" << endl;
    return true;
}

void CamController::setFlip(bool flip) {
    flip180 = flip;
}

bool CamController::isFlipped() const {
    return flip180;
}

bool CamController::captureFrame() {
    if (useRpiCam) {
        int frameSize = frameWidth * frameHeight * 3 / 2; // YUV420
        std::vector<uint8_t> buffer(frameSize);
        
        // Read complete frame in one go
        size_t totalRead = 0;
        while (totalRead < frameSize) {
            size_t bytesRead = fread(buffer.data() + totalRead, 1, 
                                    frameSize - totalRead, cameraStream);
            
            if (bytesRead == 0) {
                std::cerr << "Stream ended or error" << std::endl;
                return false;
            }
            
            totalRead += bytesRead;
        }
        
        {
            std::lock_guard<std::mutex> lock(frameMutex);
            cv::Mat yuvFrame(frameHeight * 3 / 2, frameWidth, CV_8UC1, buffer.data());
            cv::cvtColor(yuvFrame, currentFrame, cv::COLOR_YUV2BGR_I420);
            
            if (flip180) {
                cv::flip(currentFrame, currentFrame, -1);
            }
        }
        
    } else {
        cap >> currentFrame;
        
        if (currentFrame.empty()) {
            return false;
        }
        
        if (flip180) {
            cv::flip(currentFrame, currentFrame, -1);
        }
    }
    
    detectRedPixels();
    detectedObjects = findRedObjects();
    
    return true;
}

void CamController::detectRedPixels() {
    Mat hsv, mask1, mask2;
    cvtColor(currentFrame, hsv, COLOR_BGR2HSV);
    
    // Red color wraps around in HSV, so we need two ranges
    inRange(hsv, redLowerMin, redLowerMax, mask1);
    inRange(hsv, redUpperMin, redUpperMax, mask2);
    
    redMask = mask1 | mask2;
}

vector<CamController::RedObject> CamController::findRedObjects() {
    vector<RedObject> objects;
    
    vector<vector<Point>> contours;
    findContours(redMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    for (const auto& contour : contours) {
        double area = contourArea(contour);
        
        if (area > minArea) {
            RedObject obj;
            obj.area = area;
            obj.boundingBox = boundingRect(contour);
            
            Moments m = moments(contour);
            obj.center = Point2f(m.m10 / m.m00, m.m01 / m.m00);
            
            objects.push_back(obj);
        }
    }
    
    return objects;
}

CamController::Direction CamController::getDirection() {
    return analyzeRedObjects();
}

CamController::Direction CamController::analyzeRedObjects() {
    Direction dir;
    dir.objectCount = detectedObjects.size();
    
    if (detectedObjects.empty()) {
        dir.angle = 0;
        dir.command = "NO RED DETECTED";
        dir.distance = 0;
        return dir;
    }
    
    RedObject* targetObject = nullptr;
    
    // Choose target based on strategy
    if (strategy == TrackingStrategy::LARGEST) {
        targetObject = &(*max_element(detectedObjects.begin(), detectedObjects.end(),
            [](const RedObject& a, const RedObject& b) {
                return a.area < b.area;
            }));
    } 
    else if (strategy == TrackingStrategy::CLOSEST_TO_CENTER) {
        int frameCenterX = currentFrame.cols / 2;
        targetObject = &(*min_element(detectedObjects.begin(), detectedObjects.end(),
            [frameCenterX](const RedObject& a, const RedObject& b) {
                return abs(a.center.x - frameCenterX) < 
                       abs(b.center.x - frameCenterX);
            }));
    }
    else if (strategy == TrackingStrategy::LEFTMOST) {
        targetObject = &(*min_element(detectedObjects.begin(), detectedObjects.end(),
            [](const RedObject& a, const RedObject& b) {
                return a.center.x < b.center.x;
            }));
    }
    
    // Calculate direction
    int frameCenterX = currentFrame.cols / 2;
    int offsetX = targetObject->center.x - frameCenterX;
    
    double maxOffset = currentFrame.cols / 2.0;
    dir.angle = (offsetX / maxOffset) * 90.0;
    dir.distance = abs(offsetX) / maxOffset;
    
    if (abs(dir.angle) < deadOnThreshold) {
        dir.command = "FORWARD";
    } else if (dir.angle > 0) {
        dir.command = "RIGHT " + to_string((int)abs(dir.angle)) + " degrees";
    } else {
        dir.command = "LEFT " + to_string((int)abs(dir.angle)) + " degrees";
    }
    
    return dir;
}

vector<CamController::RedObject> CamController::getDetectedObjects() const {
    return detectedObjects;
}

void CamController::setTrackingStrategy(TrackingStrategy newStrategy) {
    strategy = newStrategy;
}

CamController::TrackingStrategy CamController::getTrackingStrategy() const {
    return strategy;
}

void CamController::setMinArea(double newMinArea) {
    minArea = newMinArea;
}

double CamController::getMinArea() const {
    return minArea;
}

void CamController::setDeadOnThreshold(double threshold) {
    deadOnThreshold = threshold;
}

double CamController::getDeadOnThreshold() const {
    return deadOnThreshold;
}

Mat CamController::getFrame() const {
    return currentFrame.clone();
}

Mat CamController::getFrameWithVisualization() {
    Mat visualFrame = currentFrame.clone();
    drawVisualization(visualFrame);
    return visualFrame;
}

void CamController::drawVisualization(Mat& frame) {
    if (detectedObjects.empty()) {
        return;
    }
    
    // Find target object based on current strategy
    RedObject* targetObject = nullptr;
    
    if (strategy == TrackingStrategy::LARGEST) {
        targetObject = &(*max_element(detectedObjects.begin(), detectedObjects.end(),
            [](const RedObject& a, const RedObject& b) {
                return a.area < b.area;
            }));
    } 
    else if (strategy == TrackingStrategy::CLOSEST_TO_CENTER) {
        int frameCenterX = frame.cols / 2;
        targetObject = &(*min_element(detectedObjects.begin(), detectedObjects.end(),
            [frameCenterX](const RedObject& a, const RedObject& b) {
                return abs(a.center.x - frameCenterX) < 
                       abs(b.center.x - frameCenterX);
            }));
    }
    else if (strategy == TrackingStrategy::LEFTMOST) {
        targetObject = &(*min_element(detectedObjects.begin(), detectedObjects.end(),
            [](const RedObject& a, const RedObject& b) {
                return a.center.x < b.center.x;
            }));
    }
    
    // Draw all objects
    for (size_t i = 0; i < detectedObjects.size(); i++) {
        Scalar color = (&detectedObjects[i] == targetObject) ? 
                       Scalar(0, 255, 0) : Scalar(128, 128, 128);
        
        circle(frame, detectedObjects[i].center, 10, color, 2);
        rectangle(frame, detectedObjects[i].boundingBox, color, 2);
        
        string label = "Cup " + to_string(i + 1);
        if (&detectedObjects[i] == targetObject) label += " (TARGET)";
        putText(frame, label, 
                Point(detectedObjects[i].boundingBox.x, 
                      detectedObjects[i].boundingBox.y - 10),
                FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
    }
    
    // Draw center point and line to target
    int frameCenterX = frame.cols / 2;
    int frameCenterY = frame.rows / 2;
    circle(frame, Point(frameCenterX, frameCenterY), 5, Scalar(255, 0, 0), -1);
    
    if (targetObject) {
        line(frame, Point(frameCenterX, frameCenterY), 
             targetObject->center, Scalar(255, 255, 0), 2);
    }
}

void CamController::setRedRangeLower(int hueMin, int hueMax, int satMin, int valMin) {
    redLowerMin = Scalar(hueMin, satMin, valMin);
    redLowerMax = Scalar(hueMax, 255, 255);
}

void CamController::setRedRangeUpper(int hueMin, int hueMax, int satMin, int valMin) {
    redUpperMin = Scalar(hueMin, satMin, valMin);
    redUpperMax = Scalar(hueMax, 255, 255);
}

bool CamController::isOpened() const {
    if (useRpiCam) {
        return cameraStream != nullptr;
    }
    return cap.isOpened();
}

void CamController::release() {
    if (useRpiCam && cameraStream != nullptr) {
        pclose(cameraStream);
        cameraStream = nullptr;
    } else if (cap.isOpened()) {
        cap.release();
    }
}


void CamController::enableStreaming(bool enable, int port) {
    if (enable && !streamingEnabled) {
        streamPort = port;
        streamingEnabled = true;
        streamThread = std::thread(&CamController::streamingLoop, this);
        std::cout << "Streaming enabled on http://<your-pi-ip>:" << port << std::endl;
    } else if (!enable && streamingEnabled) {
        streamingEnabled = false;
        if (streamThread.joinable()) {
            streamThread.join();
        }
        std::cout << "Streaming disabled" << std::endl;
    }
}

bool CamController::isStreamingEnabled() const {
    return streamingEnabled;
}

int CamController::getStreamPort() const {
    return streamPort;
}

void CamController::streamingLoop() {
    // Create socket
    int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return;
    }
    
    // Allow reuse of address
    int opt = 1;
    setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    // Bind to port
    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(streamPort);
    
    if (bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "Failed to bind to port " << streamPort << std::endl;
        close(serverSocket);
        return;
    }
    
    // Listen for connections
    listen(serverSocket, 1);
    std::cout << "Stream server listening on port " << streamPort << std::endl;
    
    while (streamingEnabled) {
        // Accept client connection
        struct sockaddr_in clientAddr;
        socklen_t clientLen = sizeof(clientAddr);
        int clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &clientLen);
        
        if (clientSocket < 0) {
            continue;
        }
        
        std::cout << "Client connected to stream" << std::endl;
        
        // Send HTTP header
        std::string header = 
            "HTTP/1.0 200 OK\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
            "\r\n";
        send(clientSocket, header.c_str(), header.length(), 0);
        
        // Stream frames
        while (streamingEnabled) {
            cv::Mat frame;
            
            // Get current frame with visualization
            {
                std::lock_guard<std::mutex> lock(frameMutex);
                if (currentFrame.empty()) {
                    break;
                }
                frame = getFrameWithVisualization();
            }
            
            // Encode frame as JPEG
            std::vector<uchar> buffer;
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
            cv::imencode(".jpg", frame, buffer, params);
            
            // Send frame
            std::string part = "--frame\r\n"
                             "Content-Type: image/jpeg\r\n"
                             "Content-Length: " + std::to_string(buffer.size()) + "\r\n"
                             "\r\n";
            
            if (send(clientSocket, part.c_str(), part.length(), 0) < 0) {
                break;
            }
            
            if (send(clientSocket, buffer.data(), buffer.size(), 0) < 0) {
                break;
            }
            
            if (send(clientSocket, "\r\n", 2, 0) < 0) {
                break;
            }
            
            // Control frame rate (~15 FPS)
            usleep(66000);
        }
        
        close(clientSocket);
        std::cout << "Client disconnected from stream" << std::endl;
    }
    
    close(serverSocket);
}
