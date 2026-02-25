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
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>


class CamController {
public:
    
    enum class TrackingStrategy {
        LARGEST,
        CLOSEST_TO_CENTER,
        LEFTMOST
    };
    
    struct RedObject {
        cv::Point2f center;
        double area;
        cv::Rect boundingBox;
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
    void setRedRangeLower(int hueMin, int hueMax, int satMin = 100, int valMin = 100);
    void setRedRangeUpper(int hueMin, int hueMax, int satMin = 100, int valMin = 100);
    bool isOpened() const;
    void release();
    void setFlip(bool flip);
    bool isFlipped() const;


    void enableStreaming(bool enable, int port = 8080);
    bool isStreamingEnabled() const;
    int getStreamPort() const;
    
private:
   
    cv::VideoCapture cap;
    cv::Mat currentFrame;
    cv::Mat redMask;
    std::vector<RedObject> detectedObjects;
    
    int cameraIndex;
    int frameWidth;
    int frameHeight;
    double minArea;
    double deadOnThreshold;
    
    cv::Scalar redLowerMin;
    cv::Scalar redLowerMax;
    cv::Scalar redUpperMin;
    cv::Scalar redUpperMax;
    
    TrackingStrategy strategy;
    
    FILE* cameraStream;
    bool useRpiCam;
    
    std::atomic<bool> streamingEnabled;
    int streamPort;
    std::thread streamThread;
    std::mutex frameMutex;
    cv::Mat streamFrame;
    
    void detectRedPixels();
    std::vector<RedObject> findRedObjects();
    Direction analyzeRedObjects();
    void drawVisualization(cv::Mat& frame);
    bool flip180;
    void streamingLoop();
};

#endif // CAM_CONTROLLER_H
