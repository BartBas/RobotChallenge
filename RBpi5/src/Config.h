#ifndef CONFIG_H
#define CONFIG_H

#include <string>
#include <vector>

/**
 * @file Config.h
 * @brief Robot configuration structures and config-file loader declaration.
 *
 * @details
 * Defines three plain-data structs that together describe every tunable
 * parameter of the robot, plus the `loadConfig()` function that populates
 * a `RobotConfig` from a key=value text file.
 *
 * ### Structs
 * | Struct        | Purpose                                                        |
 * |---------------|----------------------------------------------------------------|
 * | ColorConfig   | HSV range pair (lower + upper band) for one tracked colour     |
 * | ExcludeZone   | Angular range (degrees) that the lidar should ignore           |
 * | RobotConfig   | Aggregates all hardware, vision, and brain tuning parameters   |
 *
 * ### Config File Format
 * Plain text, one `key = value` pair per line. Lines beginning with `#` (or
 * containing `#` mid-line) are treated as comments and ignored. Unknown keys
 * produce a warning on stderr but do not abort loading.
 *
 * Repeatable keys:
 * - `color  <name> <lHueMin> <lHueMax> <lSatMin> <lValMin> <uHueMin> <uHueMax> <uSatMin> <uValMin>`
 * - `lidar_exclude  <angleMin> <angleMax>`
 *
 * ### Key Reference
 * | Key                        | Type   | Default       | Description                                             |
 * |----------------------------|--------|---------------|---------------------------------------------------------|
 * | lidar_port                 | string | /dev/ttyUSB0  | Serial device for the lidar                             |
 * | motor_port                 | string | /dev/ttyUSB1  | Serial device for the motor controller                  |
 * | flip_lidar                 | bool   | false         | Rotate lidar readings 180° if mounted inverted          |
 * | flip_camera                | bool   | false         | Flip camera image vertically                            |
 * | cam_width                  | int    | 640           | Capture width in pixels                                 |
 * | cam_height                 | int    | 480           | Capture height in pixels                                |
 * | cam_fps                    | int    | 30            | Capture frame rate                                      |
 * | stream_port                | int    | 8081          | MJPEG stream port                                       |
 * | web_port                   | int    | 8080          | HTTP dashboard port                                     |
 * | ws_port                    | int    | 8082          | WebSocket manual-control port                           |
 * | min_area                   | double | 200.0         | Minimum contour area (px²) to consider a detection      |
 * | dead_on_thresh             | double | 5.0           | Angle (°) within which target is considered "dead on"   |
 * | brain_hz                   | int    | 10            | Brain tick rate (Hz)                                    |
 * | brain_clear_dist           | float  | 0.6           | Obstacle clearance distance (m) that triggers AVOID     |
 * | brain_chase_speed          | int    | 45            | Motor speed (0–100) in CHASE state                      |
 * | brain_seek_speed           | int    | 30            | Motor speed (0–100) in SEEK state                       |
 * | brain_avoid_speed          | int    | 35            | Motor speed (0–100) in AVOID state                      |
 * | brain_front_arc            | float  | 40.0          | Half-angle (°) of the front obstacle detection arc      |
 * | brain_collect_dist         | double | 8000.0        | Contour area (px²) threshold to switch CHASE → COLLECT  |
 * | brain_collect_x_min        | float  | 0.55          | Left edge of collection alignment window (0–1)          |
 * | brain_collect_x_max        | float  | 0.75          | Right edge of collection alignment window (0–1)         |
 * | brain_sidestep_speed       | int    | 25            | Motor speed (0–100) during Phase A side-step            |
 * | brain_drive_speed          | int    | 30            | Motor speed (0–100) during Phase B blind drive          |
 * | brain_drive_over_ms        | int    | 1500          | Duration (ms) of the Phase B blind forward drive        |
 * | cam_elevated_area_thresh   | double | 3000.0        | Area (px²) above which the elevated-object filter fires |
 * | cam_elevated_y_thresh      | float  | 0.40          | Normalised Y above which large blobs are rejected       |
 *
 * ### Usage
 * @code
 * RobotConfig cfg;
 * loadConfig("config.txt", cfg);
 * @endcode
 */

struct ColorConfig {
    std::string name;
    int lHueMin, lHueMax, lSatMin, lValMin;
    int uHueMin, uHueMax, uSatMin, uValMin;
};

struct ExcludeZone {
    float angleMin;
    float angleMax;
};

struct RobotConfig {
    std::string lidarPort   = "/dev/ttyUSB0";
    std::string motorPort   = "/dev/ttyUSB1";
    bool        flipLidar   = false;
    bool        flipCamera  = false;

    std::vector<ExcludeZone> lidarExclude;

    int  camWidth   = 640;
    int  camHeight  = 480;
    int  camFps     = 30;
    int  streamPort = 8081;

    std::vector<ColorConfig> colors;

    double minArea      = 200.0;
    double deadOnThresh = 5.0;

    int webPort = 8080;
    int wsPort  = 8082;

    int   brainHz         = 10;
    float brainClearDist  = 0.6f;
    int   brainChaseSpeed = 45;
    int   brainSeekSpeed  = 30;
    int   brainAvoidSpeed = 35;
    float brainFrontArc   = 40.0f;

    double brainCollectDist  = 8000.0;
    float  brainCollectXMin  = 0.55f;
    float  brainCollectXMax  = 0.75f;
    int    brainSidestepSpeed = 25;
    int    brainDriveSpeed    = 30;
    int    brainDriveOverMs   = 1500;

    double camElevatedAreaThresh = 3000.0;
    float  camElevatedYThresh    = 0.40f;
};

bool loadConfig(const std::string& path, RobotConfig& cfg);

#endif // CONFIG_H