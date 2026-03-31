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
    // Pre-allocate masks once — reused every frame, no heap churn
    combinedMask_.create(height, width, CV_8UC1);
    tmpMask_.create(height, width, CV_8UC1);

    // Default red colour
    addTrackedColor("red",
                    0, 10, 100, 100,
                    170, 180, 100, 100);
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
    if (!cameraStream) { cerr << "Failed to start rpicam-vid\n"; return false; }

    useRpiCam = true;
    cout << "Camera initialized successfully\n";
    return true;
}

void CamController::setFlip(bool flip) { flip180 = flip; }
bool CamController::isFlipped() const  { return flip180; }

void CamController::setCollectionZone(float xMin, float xMax) {
    collectXMin_ = xMin;
    collectXMax_ = xMax;
}

bool CamController::addTrackedColor(const std::string& name,
                                     int lHueMin, int lHueMax, int lSatMin, int lValMin,
                                     int uHueMin, int uHueMax, int uSatMin, int uValMin)
{
    if (trackedColors.size() >= MAX_TRACKED_COLORS &&
        trackedColors.find(name) == trackedColors.end()) {
        cerr << "Max tracked colors reached, cannot add '" << name << "'\n";
        return false;
    }
    ColorRange cr;
    cr.name     = name;
    cr.lowerMin = cv::Scalar(lHueMin, lSatMin, lValMin);
    cr.lowerMax = cv::Scalar(lHueMax, 255,     255    );
    cr.upperMin = cv::Scalar(uHueMin, uSatMin, uValMin);
    cr.upperMax = cv::Scalar(uHueMax, 255,     255    );
    trackedColors[name] = cr;
    cout << "Added tracked color: " << name << "\n";
    return true;
}

bool CamController::removeTrackedColor(const std::string& name) {
    auto it = trackedColors.find(name);
    if (it == trackedColors.end()) { cerr << "Color '" << name << "' not found\n"; return false; }
    trackedColors.erase(it);
    cout << "Removed tracked color: " << name << "\n";
    return true;
}

bool CamController::hasColor(const std::string& name) const {
    return trackedColors.count(name) > 0;
}
int CamController::getColorCount() const { return (int)trackedColors.size(); }

std::vector<std::string> CamController::getColorNames() const {
    std::vector<std::string> v;
    v.reserve(trackedColors.size());
    for (const auto& p : trackedColors) v.push_back(p.first);
    return v;
}

void CamController::clearAllColors() {
    trackedColors.clear();
    cout << "Cleared all tracked colors.\n";
}

void CamController::setRedRangeLower(int hMin, int hMax, int sMin, int vMin) {
    if (!hasColor("red")) return;
    trackedColors["red"].lowerMin = Scalar(hMin, sMin, vMin);
    trackedColors["red"].lowerMax = Scalar(hMax, 255, 255);
}
void CamController::setRedRangeUpper(int hMin, int hMax, int sMin, int vMin) {
    if (!hasColor("red")) return;
    trackedColors["red"].upperMin = Scalar(hMin, sMin, vMin);
    trackedColors["red"].upperMax = Scalar(hMax, 255, 255);
}

// ─────────────────────────────────────────────────────────────────────────────

bool CamController::captureFrame()
{
    if (useRpiCam) {
        int frameSize = frameWidth * frameHeight * 3 / 2;   // YUV420
        std::vector<uint8_t> buf(frameSize);

        size_t total = 0;
        while (total < (size_t)frameSize) {
            size_t n = fread(buf.data() + total, 1, frameSize - total, cameraStream);
            if (n == 0) { cerr << "Stream ended or error\n"; return false; }
            total += n;
        }

        {
            std::lock_guard<std::mutex> lk(frameMutex);
            cv::Mat yuv(frameHeight * 3 / 2, frameWidth, CV_8UC1, buf.data());
            cv::cvtColor(yuv, currentFrame, cv::COLOR_YUV2BGR_I420);
            if (flip180) cv::flip(currentFrame, currentFrame, -1);
        }

    } else {
        cap >> currentFrame;
        if (currentFrame.empty()) return false;
        if (flip180) cv::flip(currentFrame, currentFrame, -1);
    }

    detectColorPixels();
    detectedObjects = findColorObjects();
    return true;
}

// ── detectColorPixels ────────────────────────────────────────────────────────
//  One HSV conversion, one combined mask — no per-frame allocations.
void CamController::detectColorPixels()
{
    cv::cvtColor(currentFrame, hsvFrame_, cv::COLOR_BGR2HSV);
    combinedMask_.setTo(0);

    for (const auto& kv : trackedColors) {
        const ColorRange& cr = kv.second;
        cv::inRange(hsvFrame_, cr.lowerMin, cr.lowerMax, tmpMask_);
        cv::bitwise_or(combinedMask_, tmpMask_, combinedMask_);
        cv::inRange(hsvFrame_, cr.upperMin, cr.upperMax, tmpMask_);
        cv::bitwise_or(combinedMask_, tmpMask_, combinedMask_);
    }
}

// ── findColorObjects ─────────────────────────────────────────────────────────
//  Single findContours on the combined mask — no .clone(), no per-colour loops.
//  Colour name resolved by a cheap HSV pixel lookup at the contour centroid.
std::vector<CamController::RedObject> CamController::findColorObjects()
{
    std::vector<RedObject> objects;
    std::vector<std::vector<cv::Point>> contours;

    // RETR_EXTERNAL does not modify the source mat so no .clone() needed
    cv::findContours(combinedMask_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    objects.reserve(contours.size());

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < minArea) continue;

        RedObject obj;
        obj.area        = area;
        obj.boundingBox = cv::boundingRect(contour);

        cv::Moments m = cv::moments(contour);
        obj.center = (m.m00 > 0)
            ? cv::Point2f((float)(m.m10 / m.m00), (float)(m.m01 / m.m00))
            : cv::Point2f(obj.boundingBox.x + obj.boundingBox.width  * 0.5f,
                          obj.boundingBox.y + obj.boundingBox.height * 0.5f);

        // Identify colour: check HSV pixel at centroid against each range
        obj.colorName = "unknown";
        if (!hsvFrame_.empty()) {
            int cx = std::clamp((int)obj.center.x, 0, hsvFrame_.cols - 1);
            int cy = std::clamp((int)obj.center.y, 0, hsvFrame_.rows - 1);
            cv::Vec3b px = hsvFrame_.at<cv::Vec3b>(cy, cx);

            for (const auto& kv : trackedColors) {
                const ColorRange& cr = kv.second;
                auto hit = [&](const cv::Scalar& lo, const cv::Scalar& hi) {
                    return px[0] >= lo[0] && px[0] <= hi[0]
                        && px[1] >= lo[1] && px[1] <= hi[1]
                        && px[2] >= lo[2] && px[2] <= hi[2];
                };
                if (hit(cr.lowerMin, cr.lowerMax) || hit(cr.upperMin, cr.upperMax)) {
                    obj.colorName = kv.first;
                    break;
                }
            }
        }

        objects.push_back(std::move(obj));
    }

    return objects;
}

// ─────────────────────────────────────────────────────────────────────────────

CamController::Direction CamController::getDirection() {
    return analyzeColorObjects();
}

CamController::Direction CamController::analyzeColorObjects()
{
    Direction dir;
    dir.objectCount = (int)detectedObjects.size();

    if (detectedObjects.empty()) {
        dir.angle = 0; dir.distance = 0; dir.command = "NO TARGET";
        return dir;
    }

    const RedObject* tgt = nullptr;

    if (strategy == TrackingStrategy::LARGEST) {
        tgt = &(*std::max_element(detectedObjects.begin(), detectedObjects.end(),
            [](const RedObject& a, const RedObject& b){ return a.area < b.area; }));
    }
    else if (strategy == TrackingStrategy::CLOSEST_TO_CENTER) {
        float cx = frameWidth * 0.5f;
        tgt = &(*std::min_element(detectedObjects.begin(), detectedObjects.end(),
            [cx](const RedObject& a, const RedObject& b){
                return std::abs(a.center.x - cx) < std::abs(b.center.x - cx);
            }));
    }
    else if (strategy == TrackingStrategy::LEFTMOST) {
        tgt = &(*std::min_element(detectedObjects.begin(), detectedObjects.end(),
            [](const RedObject& a, const RedObject& b){ return a.center.x < b.center.x; }));
    }
    else if (strategy == TrackingStrategy::LOWEST) {
        tgt = &(*std::max_element(detectedObjects.begin(), detectedObjects.end(),
            [](const RedObject& a, const RedObject& b){ return a.center.y < b.center.y; }));
    }

    if (!tgt) { dir.angle = 0; dir.distance = 0; dir.command = "NO TARGET"; return dir; }

    double normX  = (tgt->center.x - frameWidth * 0.5) / (frameWidth * 0.5);
    dir.angle    = normX * 90.0;
    dir.distance = normX;

    if (std::abs(dir.angle) <= deadOnThreshold)
        dir.command = "FORWARD (" + tgt->colorName + ")";
    else if (dir.angle > 0)
        dir.command = "RIGHT " + std::to_string((int)std::abs(dir.angle)) + " degrees (" + tgt->colorName + ")";
    else
        dir.command = "LEFT "  + std::to_string((int)std::abs(dir.angle)) + " degrees (" + tgt->colorName + ")";

    return dir;
}

std::vector<CamController::RedObject> CamController::getDetectedObjects() const {
    return detectedObjects;
}

void   CamController::setTrackingStrategy(TrackingStrategy s) { strategy = s; }
CamController::TrackingStrategy CamController::getTrackingStrategy() const { return strategy; }
void   CamController::setMinArea(double a)          { minArea = a; }
double CamController::getMinArea() const            { return minArea; }
void   CamController::setDeadOnThreshold(double t)  { deadOnThreshold = t; }
double CamController::getDeadOnThreshold() const    { return deadOnThreshold; }
cv::Mat CamController::getFrame() const             { return currentFrame.clone(); }

cv::Mat CamController::getFrameWithVisualization() {
    cv::Mat vis = currentFrame.clone();
    drawVisualization(vis);
    return vis;
}

void CamController::drawVisualization(cv::Mat& frame)
{
    // ── Collection zone guide lines ───────────────────────────────────────
    {
        const cv::Scalar purple(200, 0, 200);
        int xL = (int)(collectXMin_ * frame.cols);
        int xR = (int)(collectXMax_ * frame.cols);
        cv::line(frame, {xL, 0}, {xL, frame.rows - 1}, purple, 2);
        cv::line(frame, {xR, 0}, {xR, frame.rows - 1}, purple, 2);
        cv::putText(frame, "COLLECT ZONE", {xL + 4, 18},
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, purple, 1);
    }

    if (detectedObjects.empty()) return;

    // ── Pick target ───────────────────────────────────────────────────────
    const RedObject* tgt = nullptr;
    if (strategy == TrackingStrategy::LARGEST) {
        tgt = &(*std::max_element(detectedObjects.begin(), detectedObjects.end(),
            [](const RedObject& a, const RedObject& b){ return a.area < b.area; }));
    }
    else if (strategy == TrackingStrategy::CLOSEST_TO_CENTER) {
        float cx = frame.cols * 0.5f;
        tgt = &(*std::min_element(detectedObjects.begin(), detectedObjects.end(),
            [cx](const RedObject& a, const RedObject& b){
                return std::abs(a.center.x - cx) < std::abs(b.center.x - cx);
            }));
    }
    else if (strategy == TrackingStrategy::LEFTMOST) {
        tgt = &(*std::min_element(detectedObjects.begin(), detectedObjects.end(),
            [](const RedObject& a, const RedObject& b){ return a.center.x < b.center.x; }));
    }
    else if (strategy == TrackingStrategy::LOWEST) {
        tgt = &(*std::max_element(detectedObjects.begin(), detectedObjects.end(),
            [](const RedObject& a, const RedObject& b){ return a.center.y < b.center.y; }));
    }

    // ── Draw objects ──────────────────────────────────────────────────────
    for (size_t i = 0; i < detectedObjects.size(); ++i) {
        bool isT = (&detectedObjects[i] == tgt);
        cv::Scalar col = isT ? cv::Scalar(0, 255, 0) : cv::Scalar(128, 128, 128);
        cv::circle(frame, detectedObjects[i].center, 10, col, 2);
        cv::rectangle(frame, detectedObjects[i].boundingBox, col, 2);
        std::string lbl = detectedObjects[i].colorName + " " + std::to_string(i + 1);
        if (isT) lbl += " (TARGET)";
        cv::putText(frame, lbl,
                    {detectedObjects[i].boundingBox.x,
                     detectedObjects[i].boundingBox.y - 10},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, col, 2);
    }

    // ── Centre dot + line to target ───────────────────────────────────────
    cv::Point fc(frame.cols / 2, frame.rows / 2);
    cv::circle(frame, fc, 5, cv::Scalar(255, 0, 0), -1);

    if (tgt) {
        cv::line(frame, fc, tgt->center, cv::Scalar(255, 255, 0), 2);

        float normX = tgt->center.x / (float)frame.cols;
        if (normX >= collectXMin_ && normX <= collectXMax_) {
            int xL = (int)(collectXMin_ * frame.cols);
            int xR = (int)(collectXMax_ * frame.cols);
            cv::Mat overlay = frame.clone();
            cv::rectangle(overlay, {xL, 0}, {xR, frame.rows - 1},
                          cv::Scalar(0, 200, 0), cv::FILLED);
            cv::addWeighted(overlay, 0.15, frame, 0.85, 0, frame);
            cv::putText(frame, "ALIGNED", {xL + 4, 36},
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }
    }
}

bool CamController::isOpened() const {
    return useRpiCam ? cameraStream != nullptr : cap.isOpened();
}

void CamController::release() {
    if (useRpiCam && cameraStream) { pclose(cameraStream); cameraStream = nullptr; }
    else if (cap.isOpened()) cap.release();
}

void CamController::enableStreaming(bool enable, int port) {
    if (enable && !streamingEnabled) {
        streamPort = port; streamingEnabled = true;
        streamThread = std::thread(&CamController::streamingLoop, this);
        cout << "Streaming enabled on http://<your-pi-ip>:" << port << "\n";
    } else if (!enable && streamingEnabled) {
        streamingEnabled = false;
        if (streamThread.joinable()) streamThread.join();
        cout << "Streaming disabled\n";
    }
}

bool CamController::isStreamingEnabled() const { return streamingEnabled; }
int  CamController::getStreamPort()       const { return streamPort; }

void CamController::streamingLoop()
{
    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) { cerr << "Failed to create socket\n"; return; }

    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(streamPort);

    if (bind(srv, (sockaddr*)&addr, sizeof(addr)) < 0) {
        cerr << "Failed to bind to port " << streamPort << "\n";
        close(srv); return;
    }

    listen(srv, 1);
    cout << "Stream server listening on port " << streamPort << "\n";

    while (streamingEnabled) {
        sockaddr_in cli{}; socklen_t cliLen = sizeof(cli);
        int fd = accept(srv, (sockaddr*)&cli, &cliLen);
        if (fd < 0) continue;
        cout << "Client connected to stream\n";

        const std::string hdr =
            "HTTP/1.0 200 OK\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
        send(fd, hdr.c_str(), hdr.size(), 0);

        while (streamingEnabled) {
            cv::Mat frame;
            {
                std::lock_guard<std::mutex> lk(frameMutex);
                if (currentFrame.empty()) break;
                frame = getFrameWithVisualization();
            }

            std::vector<uchar> buf;
            cv::imencode(".jpg", frame, buf, {cv::IMWRITE_JPEG_QUALITY, 80});

            std::string part = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                             + std::to_string(buf.size()) + "\r\n\r\n";

            if (send(fd, part.c_str(), part.size(), 0) < 0) break;
            if (send(fd, buf.data(),   buf.size(),   0) < 0) break;
            if (send(fd, "\r\n", 2,                  0) < 0) break;

            usleep(66000);  // ~15 FPS
        }

        close(fd);
        cout << "Client disconnected from stream\n";
    }

    close(srv);
}
