#ifndef CAM_CONTROLLER_H
#define CAM_CONTROLLER_H

/**
 * @file CamController.h
 * @brief Camera capture, colour tracking, and MJPEG streaming interface.
 *
 * @details
 * Provides a self-contained pipeline that goes from raw camera frames to
 * a structured `Direction` output suitable for passing directly to the
 * `Brain`. Each call to `captureFrame()` + `getDirection()` constitutes
 * one tracking cycle.
 *
 * ### Pipeline Overview
 * ```
 * captureFrame()
 *   └─ BGR → HSV conversion
 *   └─ per-colour inRange masks → combined mask
 *   └─ contour detection → RedObject list
 *   └─ elevated-object filter
 *
 * getDirection()
 *   └─ select primary target via TrackingStrategy
 *   └─ compute bearing angle and "dead-on" command
 *   └─ return Direction{angle, command, distance, objectCount}
 * ```
 *
 * ### Colour Tracking
 * Up to `MAX_TRACKED_COLORS` (3) named colours can be registered at once.
 * Each colour is defined by two HSV bands (lower and upper wrap-around
 * ranges), which are OR-combined into a single mask each frame.
 * The legacy `setRedRangeLower()` / `setRedRangeUpper()` methods are kept
 * for backwards compatibility and map to a colour named `"red"`.
 *
 * ### Tracking Strategies
 * | Strategy            | Primary target chosen by                      |
 * |---------------------|-----------------------------------------------|
 * | LARGEST             | Greatest contour area                         |
 * | CLOSEST_TO_CENTER   | Smallest horizontal distance from frame centre|
 * | LEFTMOST            | Smallest centre-x pixel coordinate           |
 * | LOWEST              | Largest centre-y pixel coordinate (near floor)|
 *
 * ### Elevated-Object Filter
 * Rejects colour blobs that are both large (area > `elevatedAreaThresh_`)
 * and positioned high in the frame (normalised Y < `elevatedYThresh_`).
 * With the camera mounted ~7 cm off the ground, real cups always appear in
 * the lower portion of the frame, so large blobs near the top are almost
 * certainly elevated furniture.
 *
 * ### Collection Zone Guide Lines
 * Two vertical guide lines drawn on the MJPEG stream at normalised X
 * positions `collectXMin_` and `collectXMax_` mark the horizontal window
 * that the `Brain` uses for COLLECT Phase A alignment.
 *
 * ### MJPEG Streaming
 * When enabled via `enableStreaming()`, a background thread serves a
 * single-client MJPEG stream on the configured port. Frames include
 * detection overlays and collection-zone guide lines.
 *
 * ### Thread Safety
 * `captureFrame()` and `getFrameWithVisualization()` both acquire
 * `frameMutex_`. Do not call them simultaneously from different threads
 * without external coordination.
 *
 * ### Usage
 * @code
 * CamController cam(0, 640, 480);
 * cam.addTrackedColor("orange", 5,20,120,80, 170,180,120,80);
 * cam.setElevatedFilter(3000.0, 0.40f);
 * cam.enableStreaming(true, 8081);
 * cam.initialize();
 *
 * while (running) {
 *     cam.captureFrame();
 *     auto dir = cam.getDirection();
 * }
 * @endcode
 */

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>


class CamController {
public:

    enum class TrackingStrategy {
        LARGEST,
        CLOSEST_TO_CENTER,
        LEFTMOST,
        LOWEST
    };

    struct ColorRange {
        std::string name;
        cv::Scalar lowerMin;
        cv::Scalar lowerMax;
        cv::Scalar upperMin;
        cv::Scalar upperMax;
    };

    struct RedObject {
        cv::Point2f center;
        double area;
        cv::Rect boundingBox;
        std::string colorName;
    };

    struct Direction {
        double angle;
        std::string command;
        double distance;
        int objectCount;
    };

    CamController(int cameraIndex = 0, int width = 640, int height = 480);
    ~CamController();

    bool initialize();
    bool captureFrame();
    Direction getDirection();
    std::vector<RedObject> getDetectedObjects() const;
    void setTrackingStrategy(TrackingStrategy strategy);
    TrackingStrategy getTrackingStrategy() const;
    void setMinArea(double minArea);
    double getMinArea() const;
    void setDeadOnThreshold(double threshold);
    double getDeadOnThreshold() const;
    cv::Mat getFrame() const;
    cv::Mat getFrameWithVisualization();
    bool isOpened() const;
    void release();
    void setFlip(bool flip);
    bool isFlipped() const;

    void  setCollectionZone(float xMin, float xMax);
    float getCollectionZoneMin() const { return collectXMin_; }
    float getCollectionZoneMax() const { return collectXMax_; }

    bool addTrackedColor(const std::string& name,
                         int lowerHueMin, int lowerHueMax, int lowerSatMin, int lowerValMin,
                         int upperHueMin, int upperHueMax, int upperSatMin, int upperValMin);
    bool removeTrackedColor(const std::string& name);
    bool hasColor(const std::string& name) const;
    int getColorCount() const;
    std::vector<std::string> getColorNames() const;
    void clearAllColors();

    void setRedRangeLower(int hueMin, int hueMax, int satMin = 100, int valMin = 100);
    void setRedRangeUpper(int hueMin, int hueMax, int satMin = 100, int valMin = 100);

    void enableStreaming(bool enable, int port = 8080);
    bool isStreamingEnabled() const;
    int getStreamPort() const;

    void setElevatedFilter(double areaThresh, float yThresh) {
        elevatedAreaThresh_ = areaThresh;
        elevatedYThresh_    = yThresh;
    }

private:
    cv::VideoCapture cap;
    cv::Mat currentFrame;
    std::vector<RedObject> detectedObjects;

    int cameraIndex;
    int frameWidth;
    int frameHeight;
    double minArea;
    double deadOnThreshold;

    std::map<std::string, ColorRange> trackedColors;
    static const int MAX_TRACKED_COLORS = 3;

    TrackingStrategy strategy;

    FILE* cameraStream;
    bool useRpiCam;

    std::atomic<bool> streamingEnabled;
    int streamPort;
    std::thread streamThread;
    std::mutex frameMutex;

    float collectXMin_ = 0.55f;
    float collectXMax_ = 0.75f;

    double elevatedAreaThresh_ = 3000.0;
    float  elevatedYThresh_    = 0.40f;

    cv::Mat hsvFrame_;
    cv::Mat combinedMask_;
    cv::Mat tmpMask_;

    bool flip180;

    void detectColorPixels();
    std::vector<RedObject> findColorObjects();
    Direction analyzeColorObjects();
    void drawVisualization(cv::Mat& frame);
    void streamingLoop();
};

#endif // CAM_CONTROLLER_H