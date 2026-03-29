#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <vector>

// ── Per-colour tracking entry ─────────────────────────────────────
struct ColorConfig {
    std::string name;
    int lHueMin, lHueMax, lSatMin, lValMin;
    int uHueMin, uHueMax, uSatMin, uValMin;
};

// ── Lidar exclusion zone (angle range to blank out) ───────────────
struct ExcludeZone {
    float angleMin;
    float angleMax;
};

// ── Full robot config ─────────────────────────────────────────────
struct RobotConfig {
    // Hardware
    std::string lidarPort   = "/dev/ttyUSB0";
    std::string motorPort   = "/dev/ttyUSB1";
    bool        flipLidar   = false;
    bool        flipCamera  = false;

    // Lidar wheel exclusion zones
    std::vector<ExcludeZone> lidarExclude;

    // Camera
    int  camWidth   = 640;
    int  camHeight  = 480;
    int  camFps     = 30;
    int  streamPort = 8081;

    // Colour tracking
    std::vector<ColorConfig> colors;

    // Vision tuning
    double minArea      = 200.0;
    double deadOnThresh = 5.0;

    // Web dashboard
    int webPort = 8080;

    // Brain
    int   brainHz         = 10;
    float brainClearDist  = 0.6f;
    int   brainChaseSpeed = 45;
    int   brainSeekSpeed  = 30;
    int   brainAvoidSpeed = 35;
    float brainFrontArc   = 40.0f;
};

// Load config from file. Returns true on success.
// Missing keys fall back to the defaults above.
bool loadConfig(const std::string& path, RobotConfig& cfg);

#endif // CONFIG_H
