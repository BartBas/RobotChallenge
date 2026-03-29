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
    , collectXMin_(0.55f)
    , collectXMax_(0.75f)
{
    // Initialize with default red color
    addTrackedColor("red",
                    0, 10, 100, 100,      // Lower red range (0-10 hue)
                    170, 180, 100, 100);  // Upper red range (170-180 hue)
}

CamController::~CamController() {
    enableStreaming(false);
    release();
}

bool CamController::initialize() {
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

void CamController::setCollectionZone(float xMin, float xMax) {
    collectXMin_ = xMin;
    collectXMax_ = xMax;
}

bool CamController::addTrackedColor(const std::string& name,
                                     int lowerHueMin, int lowerHueMax, int lowerSatMin, int lowerValMin,
                                     int upperHueMin, int upperHueMax, int upperSatMin, int upperValMin) {
    if (trackedColors.size() >= MAX_TRACKED_COLORS && trackedColors.find(name) == trackedColors.end()) {
        cerr << "Maximum number of tracked colors (" << MAX_TRACKED_COLORS << ") reached. Cannot add '" << name << "'." << endl;
        return false;
    }

    ColorRange range;
    range.name = name;
    range.lowerMin = cv::Scalar(lowerHueMin, lowerSatMin, lowerValMin);
    range.lowerMax = cv::Scalar(lowerHueMax, 255, 255);
    range.upperMin = cv::Scalar(upperHueMin, upperSatMin, upperValMin);
    range.upperMax = cv::Scalar(upperHueMax, 255, 255);

    trackedColors[name] = range;
    cout << "Added tracked color: " << name << endl;
    return true;
}

bool CamController::removeTrackedColor(const std::string& name) {
    auto it = trackedColors.find(name);
    if (it != trackedColors.end()) {
        trackedColors.erase(it);
        cout << "Removed tracked color: " << name << endl;
        return true;
    }
    cerr << "Color '" << name << "' not found." << endl;
    return false;
}

bool CamController::hasColor(const std::string& name) const {
    return trackedColors.find(name) != trackedColors.end();
}

int CamController::getColorCount() const {
    return trackedColors.size();
}

std::vector<std::string> CamController::getColorNames() const {
    std::vector<std::string> names;
    for (const auto& pair : trackedColors) {
        names.push_back(pair.first);
    }
    return names;
}

void CamController::clearAllColors() {
    trackedColors.clear();
    cout << "Cleared all tracked colors." << endl;
}

void CamController::setRedRangeLower(int hueMin, int hueMax, int satMin, int valMin) {
    if (hasColor("red")) {
        ColorRange& redRange = trackedColors["red"];
        redRange.lowerMin = Scalar(hueMin, satMin, valMin);
        redRange.lowerMax = Scalar(hueMax, 255, 255);
    }
}

void CamController::setRedRangeUpper(int hueMin, int hueMax, int satMin, int valMin) {
    if (hasColor("red")) {
        ColorRange& redRange = trackedColors["red"];
        redRange.upperMin = Scalar(hueMin, satMin, valMin);
        redRange.upperMax = Scalar(hueMax, 255, 255);
    }
}

bool CamController::captureFrame() {
    if (useRpiCam) {
        int frameSize = frameWidth * frameHeight * 3 / 2; // YUV420
        std::vector<uint8_t> buffer(frameSize);

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

    detectColorPixels();
    detectedObjects = findColorObjects();

    return true;
}

void CamController::detectColorPixels() {
    Mat hsv;
    cvtColor(currentFrame, hsv, COLOR_BGR2HSV);

    colorMasks.clear();

    for (const auto& colorPair : trackedColors) {
        const ColorRange& range = colorPair.second;
        Mat mask1, mask2;

        inRange(hsv, range.lowerMin, range.lowerMax, mask1);
        inRange(hsv, range.upperMin, range.upperMax, mask2);

        Mat colorMask = mask1 | mask2;
        colorMasks.push_back(colorMask);
    }
}

vector<CamController::RedObject> CamController::findColorObjects() {
    vector<RedObject> objects;

    size_t colorIndex = 0;
    for (const auto& colorPair : trackedColors) {
        const string& colorName = colorPair.first;

        if (colorIndex >= colorMasks.size()) break;

        vector<vector<Point>> contours;
        findContours(colorMasks[colorIndex].clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            double area = contourArea(contour);

            if (area > minArea) {
                RedObject obj;
                obj.area = area;
                obj.boundingBox = boundingRect(contour);
                obj.colorName = colorName;

                Moments m = moments(contour);
                if (m.m00 > 0) {
                    obj.center = Point2f(m.m10 / m.m00, m.m01 / m.m00);
                } else {
                    obj.center = Point2f(obj.boundingBox.x + obj.boundingBox.width / 2,
                                        obj.boundingBox.y + obj.boundingBox.height / 2);
                }

                objects.push_back(obj);
            }
        }

        colorIndex++;
    }

    return objects;
}

CamController::Direction CamController::getDirection() {
    return analyzeColorObjects();
}

CamController::Direction CamController::analyzeColorObjects() {
    Direction dir;
    dir.objectCount = detectedObjects.size();

    if (detectedObjects.empty()) {
        dir.angle = 0;
        dir.distance = 0;
        dir.command = "NO TARGET";
        return dir;
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
        int frameCenterX = frameWidth / 2;
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
    else if (strategy == TrackingStrategy::LOWEST) {
        targetObject = &(*max_element(detectedObjects.begin(), detectedObjects.end(),
            [](const RedObject& a, const RedObject& b) {
                return a.center.y < b.center.y;
            }));
    }

    if (!targetObject) {
        dir.angle = 0; dir.distance = 0; dir.command = "NO TARGET";
        return dir;
    }

    int frameCenterX = frameWidth / 2;
    double normalizedX = (targetObject->center.x - frameCenterX) / (double)(frameWidth / 2);
    dir.angle    = normalizedX * 90.0;
    dir.distance = normalizedX;

    if (std::abs(dir.angle) <= deadOnThreshold) {
        dir.command = "FORWARD (" + targetObject->colorName + ")";
    } else if (dir.angle > 0) {
        dir.command = "RIGHT " + to_string((int)abs(dir.angle)) + " degrees (" + targetObject->colorName + ")";
    } else {
        dir.command = "LEFT "  + to_string((int)abs(dir.angle)) + " degrees (" + targetObject->colorName + ")";
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

void CamController::drawVisualization(Mat& frame)
{
    // ── 1. Purple collection-zone guide lines ─────────────────────────────
    //       Always drawn so you can tune them even without a target present.
    {
        const cv::Scalar purple(200, 0, 200);   // BGR purple
        const int lineThick = 2;

        int xLeft  = static_cast<int>(collectXMin_ * frame.cols);
        int xRight = static_cast<int>(collectXMax_ * frame.cols);

        // Full-height vertical lines
        cv::line(frame,
                 cv::Point(xLeft,  0), cv::Point(xLeft,  frame.rows - 1),
                 purple, lineThick);
        cv::line(frame,
                 cv::Point(xRight, 0), cv::Point(xRight, frame.rows - 1),
                 purple, lineThick);

        // Label at top between the lines
        cv::putText(frame, "COLLECT ZONE",
                    cv::Point(xLeft + 4, 18),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, purple, 1);
    }

    // ── 2. Detected objects ───────────────────────────────────────────────
    if (detectedObjects.empty()) return;

    // Pick target by current strategy
    RedObject* targetObject = nullptr;

    if (strategy == TrackingStrategy::LARGEST) {
        targetObject = &(*std::max_element(
            detectedObjects.begin(), detectedObjects.end(),
            [](const RedObject& a, const RedObject& b){ return a.area < b.area; }));
    }
    else if (strategy == TrackingStrategy::CLOSEST_TO_CENTER) {
        int cx = frame.cols / 2;
        targetObject = &(*std::min_element(
            detectedObjects.begin(), detectedObjects.end(),
            [cx](const RedObject& a, const RedObject& b){
                return std::abs(a.center.x - cx) < std::abs(b.center.x - cx);
            }));
    }
    else if (strategy == TrackingStrategy::LEFTMOST) {
        targetObject = &(*std::min_element(
            detectedObjects.begin(), detectedObjects.end(),
            [](const RedObject& a, const RedObject& b){ return a.center.x < b.center.x; }));
    }
    else if (strategy == TrackingStrategy::LOWEST) {
        targetObject = &(*std::max_element(
            detectedObjects.begin(), detectedObjects.end(),
            [](const RedObject& a, const RedObject& b){ return a.center.y < b.center.y; }));
    }

    // Draw all objects
    for (size_t i = 0; i < detectedObjects.size(); i++) {
        bool isTarget = (&detectedObjects[i] == targetObject);
        cv::Scalar boxColor = isTarget ? cv::Scalar(0, 255, 0)
                                       : cv::Scalar(128, 128, 128);

        cv::circle(frame, detectedObjects[i].center, 10, boxColor, 2);
        cv::rectangle(frame, detectedObjects[i].boundingBox, boxColor, 2);

        std::string label = detectedObjects[i].colorName + " " + std::to_string(i + 1);
        if (isTarget) label += " (TARGET)";
        cv::putText(frame, label,
                    cv::Point(detectedObjects[i].boundingBox.x,
                              detectedObjects[i].boundingBox.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, boxColor, 2);
    }

    // Frame-centre dot and line to target
    int fcx = frame.cols / 2;
    int fcy = frame.rows / 2;
    cv::circle(frame, cv::Point(fcx, fcy), 5, cv::Scalar(255, 0, 0), -1);

    if (targetObject) {
        cv::line(frame,
                 cv::Point(fcx, fcy),
                 targetObject->center,
                 cv::Scalar(255, 255, 0), 2);

        // ── 3. Highlight when target is inside the collection zone ──────
        float normX = targetObject->center.x / (float)frame.cols;
        if (normX >= collectXMin_ && normX <= collectXMax_) {
            // Semi-transparent green tint between the lines
            int xLeft  = static_cast<int>(collectXMin_ * frame.cols);
            int xRight = static_cast<int>(collectXMax_ * frame.cols);
            cv::Mat overlay = frame.clone();
            cv::rectangle(overlay,
                          cv::Point(xLeft, 0),
                          cv::Point(xRight, frame.rows - 1),
                          cv::Scalar(0, 200, 0), cv::FILLED);
            cv::addWeighted(overlay, 0.15, frame, 0.85, 0, frame);

            cv::putText(frame, "ALIGNED",
                        cv::Point(xLeft + 4, 36),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 255, 0), 2);
        }
    }
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
    int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return;
    }

    int opt = 1;
    setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(streamPort);

    if (bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "Failed to bind to port " << streamPort << std::endl;
        close(serverSocket);
        return;
    }

    listen(serverSocket, 1);
    std::cout << "Stream server listening on port " << streamPort << std::endl;

    while (streamingEnabled) {
        struct sockaddr_in clientAddr;
        socklen_t clientLen = sizeof(clientAddr);
        int clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &clientLen);

        if (clientSocket < 0) {
            continue;
        }

        std::cout << "Client connected to stream" << std::endl;

        std::string header =
            "HTTP/1.0 200 OK\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
            "\r\n";
        send(clientSocket, header.c_str(), header.length(), 0);

        while (streamingEnabled) {
            cv::Mat frame;

            {
                std::lock_guard<std::mutex> lock(frameMutex);
                if (currentFrame.empty()) {
                    break;
                }
                frame = getFrameWithVisualization();
            }

            std::vector<uchar> buffer;
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
            cv::imencode(".jpg", frame, buffer, params);

            std::string part = "--frame\r\n"
                             "Content-Type: image/jpeg\r\n"
                             "Content-Length: " + std::to_string(buffer.size()) + "\r\n"
                             "\r\n";

            if (send(clientSocket, part.c_str(), part.length(), 0) < 0) break;
            if (send(clientSocket, buffer.data(), buffer.size(), 0) < 0) break;
            if (send(clientSocket, "\r\n", 2, 0) < 0) break;

            usleep(66000); // ~15 FPS
        }

        close(clientSocket);
        std::cout << "Client disconnected from stream" << std::endl;
    }

    close(serverSocket);
}
