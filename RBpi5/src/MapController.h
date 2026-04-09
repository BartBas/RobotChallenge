#ifndef MAP_CONTROLLER_H
#define MAP_CONTROLLER_H

/**
 * @file MapController.h
 * @brief Scan-to-scan ICP odometry, occupancy grid mapping, and point-cloud
 *        accumulation for the cup-collecting robot.
 *
 * @details
 * Maintains a consistent 2D map of the environment by fusing successive
 * lidar scans. Each call to `update()` performs odometry estimation and
 * updates three parallel representations of the world: a pose trail, a
 * world-frame point cloud, and a 2D occupancy grid.
 *
 * ### Update Cycle (called once per lidar scan)
 * ```
 * update(scan)
 *   └─ toEigen2D()        — decode raw LidarPoints → 2D robot-frame vectors
 *   └─ icp2D()            — align current scan to previous scan (SVD-based)
 *   └─ compose transform  — accumulate relative motion into pose_
 *   └─ append to path_    — trail of Pose2D, capped at maxPath_ entries
 *   └─ project to world   — rotate/translate points by current pose
 *   └─ voxelFilterCloud() — every 20 updates, merge cloud_ into 10 cm voxels
 *   └─ raycastIntoGrid()  — Bresenham raycasting: mark FREE / OCCUPIED cells
 * ```
 *
 * ### Odometry — Scan-to-Scan ICP
 * Uses a point-to-point 2D ICP with SVD-derived rotation. Both scans are
 * downsampled to at most 200 points before matching to stay within
 * Raspberry Pi compute budget. Correspondences farther than 50 cm apart are
 * rejected. Results are sanity-checked and discarded if the estimated motion
 * exceeds 1 m or 45° between consecutive scans.
 *
 * ### Occupancy Grid
 * | Cell value | Meaning  |
 * |------------|----------|
 * | 0          | Unknown  |
 * | 1          | Free     |
 * | 2          | Occupied |
 *
 * The grid starts at 400 × 400 cells (20 × 20 m at 5 cm resolution) centred
 * on the origin and grows dynamically as the robot moves, subject to a hard
 * cap of ±20 m in each axis.
 *
 * ### Memory Management
 * The world-frame point cloud (`cloud_`) is voxel-filtered every 20 update
 * calls (~2 s at 10 Hz) using a 10 cm voxel size. Structural map fidelity is
 * preserved because the occupancy grid holds the authoritative obstacle data.
 *
 * ### Thread Safety
 * All public methods (`update`, `reset`, and all accessors) are protected by
 * an internal `std::mutex`. Accessors return copies, so the caller does not
 * need to hold any lock after the call returns.
 *
 * ### Usage
 * @code
 * MapController map(0.05, 8.0, 2000);
 *
 * // In lidar thread:
 * map.update(lidar.getLatestScan());
 *
 * // In web/display thread:
 * Pose2D  pose  = map.getPose();
 * OccGrid grid  = map.getGrid();
 * @endcode
 */

#include <vector>
#include <deque>
#include <mutex>
#include <unordered_map>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/LU>
#include "LidarController.h"

struct Pose2D {
    double x   = 0.0;
    double y   = 0.0;
    double yaw = 0.0;
};

struct OccGrid {
    std::vector<int8_t> cells;
    int    width   = 0;
    int    height  = 0;
    double res     = 0.05;
    double originX = 0.0;
    double originY = 0.0;
};

class MapController {
public:
    explicit MapController(double res_m     = 0.05,
                           double max_range = 8.0,
                           size_t max_path  = 2000);

    void update(const std::vector<LidarPoint>& scan);
    void reset();

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

    std::vector<Eigen::Vector2d> prevScan_;
    bool                         hasPrev_ = false;

    double maxRange_;
    size_t maxPath_;
    double res_;

    std::vector<Eigen::Vector2d>
        toEigen2D(const std::vector<LidarPoint>& scan) const;

    bool icp2D(const std::vector<Eigen::Vector2d>& src,
               const std::vector<Eigen::Vector2d>& ref,
               double& outDx, double& outDy, double& outDYaw,
               int maxIter = 20, double tolerance = 1e-4) const;

    static std::vector<Eigen::Vector2d>
        transform2D(const std::vector<Eigen::Vector2d>& pts,
                    double tx, double ty, double yaw);

    void raycastIntoGrid(const Pose2D& pose,
                         const std::vector<Eigen::Vector2d>& pts);
    void ensureGridCovers(double wx, double wy);
    bool worldToCell(double wx, double wy, int& col, int& row) const;

    void voxelFilterCloud();
};

#endif // MAP_CONTROLLER_H