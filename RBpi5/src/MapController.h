#ifndef MAP_CONTROLLER_H
#define MAP_CONTROLLER_H

#include <vector>
#include <deque>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/SVD>
#include "LidarController.h"
#include <Eigen/LU>

struct Pose2D {
    double x   = 0.0;
    double y   = 0.0;
    double yaw = 0.0;   // radians
};

struct OccGrid {
    std::vector<int8_t> cells;   // 0=unknown, 1=free, 2=occupied
    int    width   = 0;
    int    height  = 0;
    double res     = 0.05;       // metres per cell
    double originX = 0.0;
    double originY = 0.0;
};

class MapController {
public:
    explicit MapController(double res_m     = 0.05,
                           double max_range = 8.0,
                           size_t max_path  = 2000);

    // Feed a new lidar scan. Thread-safe.
    void update(const std::vector<LidarPoint>& scan);

    // Reset map and pose to zero
    void reset();

    // Accessors (all return copies, thread-safe)
    Pose2D                       getPose()  const;
    std::vector<Pose2D>          getPath()  const;
    std::vector<Eigen::Vector3d> getCloud() const;
    OccGrid                      getGrid()  const;

private:
    mutable std::mutex           mutex_;

    Pose2D                       pose_;
    std::deque<Pose2D>           path_;
    std::vector<Eigen::Vector3d> cloud_;
    OccGrid                      grid_;

    // Previous scan in robot-local frame (for scan-to-scan ICP)
    std::vector<Eigen::Vector2d> prevScan_;
    bool                         hasPrev_ = false;

    double maxRange_;
    size_t maxPath_;
    double res_;

    // Convert raw lidar scan → clean 2D points in robot frame
    std::vector<Eigen::Vector2d>
        toEigen2D(const std::vector<LidarPoint>& scan) const;

    // 2D point-to-point ICP (SVD-based, iterative).
    // Returns false if the scan is too small or motion too ambiguous.
    bool icp2D(const std::vector<Eigen::Vector2d>& src,
               const std::vector<Eigen::Vector2d>& ref,
               double& outDx, double& outDy, double& outDYaw,
               int maxIter = 20, double tolerance = 1e-4) const;

    // Apply a 2D rigid transform to a point set
    static std::vector<Eigen::Vector2d>
        transform2D(const std::vector<Eigen::Vector2d>& pts,
                    double tx, double ty, double yaw);

    // Occupancy grid helpers (same as before)
    void raycastIntoGrid(const Pose2D& pose,
                         const std::vector<Eigen::Vector2d>& pts);
    void ensureGridCovers(double wx, double wy);
    bool worldToCell(double wx, double wy, int& col, int& row) const;
};

#endif // MAP_CONTROLLER_H
