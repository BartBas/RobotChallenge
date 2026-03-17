#ifndef CAM_CONTROLLER_H
#define CAM_CONTROLLER_H

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
        std::string colorName;  // Which color was detected
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

    // Multi-color tracking functions
    bool addTrackedColor(const std::string& name, 
                        int lowerHueMin, int lowerHueMax, int lowerSatMin, int lowerValMin,
                        int upperHueMin, int upperHueMax, int upperSatMin, int upperValMin);
    bool removeTrackedColor(const std::string& name);
    bool hasColor(const std::string& name) const;
    int getColorCount() const;
    std::vector<std::string> getColorNames() const;
    void clearAllColors();
    
    // Legacy red color setter (deprecated but kept for compatibility)
    void setRedRangeLower(int hueMin, int hueMax, int satMin = 100, int valMin = 100);
    void setRedRangeUpper(int hueMin, int hueMax, int satMin = 100, int valMin = 100);

    void enableStreaming(bool enable, int port = 8080);
    bool isStreamingEnabled() const;
    int getStreamPort() const;
    
private:
   
    cv::VideoCapture cap;
    cv::Mat currentFrame;
    std::vector<cv::Mat> colorMasks;  // One mask per tracked color
    std::vector<RedObject> detectedObjects;
    
    int cameraIndex;
    int frameWidth;
    int frameHeight;
    double minArea;
    double deadOnThreshold;
    
    // Map of color name to color ranges
    std::map<std::string, ColorRange> trackedColors;
    static const int MAX_TRACKED_COLORS = 3;
    
    TrackingStrategy strategy;
    
    FILE* cameraStream;
    bool useRpiCam;
    
    std::atomic<bool> streamingEnabled;
    int streamPort;
    std::thread streamThread;
    std::mutex frameMutex;
    cv::Mat streamFrame;
    
    void detectColorPixels();
    std::vector<RedObject> findColorObjects();
    Direction analyzeColorObjects();
    void drawVisualization(cv::Mat& frame);
    bool flip180;
    void streamingLoop();
};

#endif // CAM_CONTROLLER_H
