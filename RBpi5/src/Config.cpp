#include "Config.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>

// Trim whitespace from both ends
static std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    size_t b = s.find_last_not_of(" \t\r\n");
    return (a == std::string::npos) ? "" : s.substr(a, b - a + 1);
}

// Case-insensitive "true"/"1"/"yes" → true
static bool parseBool(const std::string& v) {
    std::string l = v;
    std::transform(l.begin(), l.end(), l.begin(), ::tolower);
    return l == "true" || l == "1" || l == "yes";
}

bool loadConfig(const std::string& path, RobotConfig& cfg)
{
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "[Config] Cannot open " << path
                  << " — using defaults\n";
        return false;
    }

    std::string line;
    int lineNo = 0;

    while (std::getline(f, line)) {
        ++lineNo;
        auto hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        line = trim(line);
        if (line.empty()) continue;

        auto eq = line.find('=');
        if (eq == std::string::npos) {
            std::cerr << "[Config] Line " << lineNo << ": no '=' found, skipping\n";
            continue;
        }

        std::string key = trim(line.substr(0, eq));
        std::string val = trim(line.substr(eq + 1));

        if      (key == "lidar_port")              cfg.lidarPort           = val;
        else if (key == "motor_port")              cfg.motorPort           = val;
        else if (key == "flip_lidar")              cfg.flipLidar           = parseBool(val);
        else if (key == "flip_camera")             cfg.flipCamera          = parseBool(val);
        else if (key == "cam_width")               cfg.camWidth            = std::stoi(val);
        else if (key == "cam_height")              cfg.camHeight           = std::stoi(val);
        else if (key == "cam_fps")                 cfg.camFps              = std::stoi(val);
        else if (key == "stream_port")             cfg.streamPort          = std::stoi(val);
        else if (key == "min_area")                cfg.minArea             = std::stod(val);
        else if (key == "dead_on_thresh")          cfg.deadOnThresh        = std::stod(val);
        else if (key == "web_port")                cfg.webPort             = std::stoi(val);
        else if (key == "ws_port") 				   cfg.wsPort 			   = std::stoi(val);
        else if (key == "brain_hz")                cfg.brainHz             = std::stoi(val);
        else if (key == "brain_clear_dist")        cfg.brainClearDist      = std::stof(val);
        else if (key == "brain_chase_speed")       cfg.brainChaseSpeed     = std::stoi(val);
        else if (key == "brain_seek_speed")        cfg.brainSeekSpeed      = std::stoi(val);
        else if (key == "brain_avoid_speed")       cfg.brainAvoidSpeed     = std::stoi(val);
        else if (key == "brain_front_arc")         cfg.brainFrontArc       = std::stof(val);
        // Cup collection
        else if (key == "brain_collect_dist")      cfg.brainCollectDist    = std::stod(val);
        else if (key == "brain_collect_x_min")     cfg.brainCollectXMin    = std::stof(val);
        else if (key == "brain_collect_x_max")     cfg.brainCollectXMax    = std::stof(val);
        else if (key == "brain_sidestep_speed")    cfg.brainSidestepSpeed  = std::stoi(val);
        else if (key == "brain_drive_speed")       cfg.brainDriveSpeed     = std::stoi(val);
        else if (key == "brain_drive_over_ms")     cfg.brainDriveOverMs    = std::stoi(val);

        else if (key == "lidar_exclude") {
            std::istringstream ss(val);
            ExcludeZone z;
            if (ss >> z.angleMin >> z.angleMax)
                cfg.lidarExclude.push_back(z);
            else
                std::cerr << "[Config] Line " << lineNo
                          << ": bad lidar_exclude format (expected: min max)\n";
        }

        else if (key == "color") {
            std::istringstream ss(val);
            ColorConfig c;
            if (ss >> c.name
                   >> c.lHueMin >> c.lHueMax >> c.lSatMin >> c.lValMin
                   >> c.uHueMin >> c.uHueMax >> c.uSatMin >> c.uValMin) {
                cfg.colors.push_back(c);
            } else {
                std::cerr << "[Config] Line " << lineNo
                          << ": bad color format\n"
                          << "  Expected: name lHueMin lHueMax lSatMin lValMin"
                             " uHueMin uHueMax uSatMin uValMin\n";
            }
        }

        else {
            std::cerr << "[Config] Line " << lineNo
                      << ": unknown key '" << key << "'\n";
        }
    }

    std::cout << "[Config] Loaded from " << path << "\n"
              << "  " << cfg.colors.size()       << " tracked colour(s)\n"
              << "  " << cfg.lidarExclude.size() << " lidar exclusion zone(s)\n"
              << "  collect window: "
              << (int)(cfg.brainCollectXMin * 100) << "% – "
              << (int)(cfg.brainCollectXMax * 100) << "% (area trigger: "
              << cfg.brainCollectDist << " px²)\n";
    return true;
}
