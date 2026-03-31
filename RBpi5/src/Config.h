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

    // Web dashboard port and manual WS port
    int webPort = 8080;
    int wsPort = 8082;

    // Brain — general
    int   brainHz         = 10;
    float brainClearDist  = 0.6f;
    int   brainChaseSpeed = 45;
    int   brainSeekSpeed  = 30;
    int   brainAvoidSpeed = 35;
    float brainFrontArc   = 40.0f;

    // Brain — cup collection
    //   brainCollectDist  : pixel-area threshold (px²) at which the robot
    //                        switches from CHASE → COLLECT.  Tune this so it
    //                        triggers when the cup fills roughly the bottom
    //                        quarter of the frame.
    double brainCollectDist  = 8000.0;

    //   brainCollectXMin/Max : normalised (0–1) horizontal window on the camera
    //                          frame.  The robot sidestepping left until the cup
    //                          centre falls within this band, then drives forward
    //                          to scoop.  The collector is on the RIGHT side of
    //                          the robot, so the window sits right-of-centre.
    //                          Purple guide lines are drawn at these positions on
    //                          the MJPEG stream.
    float brainCollectXMin   = 0.55f;   // ~55 % from left edge
    float brainCollectXMax   = 0.75f;   // ~75 % from left edge

    //   brainSidestepSpeed : motor speed (0-100) used while side-stepping left
    int   brainSidestepSpeed = 25;

    //   brainDriveSpeed    : motor speed (0-100) used for the final drive-over
    int   brainDriveSpeed    = 30;

    //   brainDriveOverMs   : how long (milliseconds) to drive forward blindly
    //                        during the drive-over phase.  The camera loses the
    //                        cup as soon as the robot passes over it, so this
    //                        timer keeps the robot moving until the cup is fully
    //                        inside the water-wheel collector.
    //                        Start with ~1500 ms and tune from there.
    int   brainDriveOverMs   = 1500;
};

// Load config from file. Returns true on success.
// Missing keys fall back to the defaults above.
bool loadConfig(const std::string& path, RobotConfig& cfg);

#endif // CONFIG_H
