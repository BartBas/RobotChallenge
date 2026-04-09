/**
 * @file MapController.cpp
 * @brief Implementation of ICP odometry, occupancy grid mapping, and
 *        point-cloud management.
 *
 * @details
 * All heavy computation lives here. Key implementation notes:
 *
 * ### ICP Downsample Strategy
 * Both the current and previous scans are uniformly strided to at most 200
 * points before the nearest-neighbour search. This keeps the O(n²) brute-force
 * correspondence search within ~40 000 distance evaluations per iteration —
 * fast enough for 10 Hz operation on a Raspberry Pi 5.
 *
 * ### Voxel Filter
 * `voxelFilterCloud()` uses a flat `unordered_map` keyed on a 64-bit integer
 * packing the (ix, iy) voxel indices. The last point written to each cell
 * wins. At a 10 cm voxel size this caps the cloud at roughly 40 000 points
 * for a 20 × 20 m arena, keeping serialisation to the web dashboard fast.
 *
 * ### Bresenham Raycasting
 * `raycastIntoGrid()` walks from the robot cell to each hit cell using
 * integer Bresenham arithmetic. Every cell along the ray (except the
 * endpoint) is marked FREE (1); the endpoint is marked OCCUPIED (2).
 * Existing OCCUPIED cells are not overwritten by FREE rays — a cell can
 * only transition unknown → free or unknown/free → occupied.
 *
 * ### Grid Growth
 * `ensureGridCovers()` pads the grid by 5 m whenever a world point
 * approaches the current boundary. Growth clears the cell array to zero
 * (unknown), so the occupancy history is not retained across expansions.
 * The hard cap of ±20 m prevents unbounded memory use if ICP drifts.
 */

#include "MapController.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>
#include <unordered_map>

MapController::MapController(double res_m, double max_range, size_t max_path)
    : maxRange_(max_range)
    , maxPath_(max_path)
    , res_(res_m)
{
    grid_.res     = res_m;
    grid_.width   = 400;
    grid_.height  = 400;
    grid_.originX = -10.0;
    grid_.originY = -10.0;
    grid_.cells.assign(grid_.width * grid_.height, 0);
}

void MapController::reset()
{
    std::lock_guard<std::mutex> lk(mutex_);
    pose_    = {};
    path_.clear();
    cloud_.clear();
    prevScan_.clear();
    hasPrev_ = false;
    grid_.cells.assign(grid_.width * grid_.height, 0);
}

void MapController::update(const std::vector<LidarPoint>& scan)
{
    if (scan.empty()) return;

    auto pts = toEigen2D(scan);
    if (pts.size() < 100) return;

    if (!hasPrev_) {
        std::lock_guard<std::mutex> lk(mutex_);
        prevScan_ = pts;
        hasPrev_  = true;
        std::cout << "[MapController] First scan stored, odometry starts next frame\n";
        return;
    }

    double dx = 0, dy = 0, dYaw = 0;
    bool ok = icp2D(pts, prevScan_, dx, dy, dYaw);

    std::lock_guard<std::mutex> lk(mutex_);

    if (ok) {
        double c = std::cos(pose_.yaw);
        double s = std::sin(pose_.yaw);
        pose_.x   += c * dx - s * dy;
        pose_.y   += s * dx + c * dy;
        pose_.yaw += dYaw;
        while (pose_.yaw >  M_PI) pose_.yaw -= 2.0 * M_PI;
        while (pose_.yaw < -M_PI) pose_.yaw += 2.0 * M_PI;
    }

    prevScan_ = pts;

    path_.push_back(pose_);
    if (path_.size() > maxPath_) path_.pop_front();

    double c = std::cos(pose_.yaw);
    double s = std::sin(pose_.yaw);
    for (const auto& p : pts) {
        double wx = pose_.x + c * p.x() - s * p.y();
        double wy = pose_.y + s * p.x() + c * p.y();
        cloud_.emplace_back(wx, wy, 0.0);
    }

    static int cloudUpdateCount = 0;
    if (++cloudUpdateCount >= 20) {
        cloudUpdateCount = 0;
        voxelFilterCloud();
    }

    raycastIntoGrid(pose_, pts);
}

void MapController::voxelFilterCloud()
{
    const double voxel = 0.10;

    std::unordered_map<uint64_t, Eigen::Vector3d> cells;
    cells.reserve(cloud_.size());

    for (const auto& p : cloud_) {
        int64_t ix = static_cast<int64_t>(std::floor(p.x() / voxel));
        int64_t iy = static_cast<int64_t>(std::floor(p.y() / voxel));
        uint64_t key = ((uint64_t)((ix + 0x7FFFFFFF) & 0xFFFFFFFF) << 32) |
                        (uint64_t)((iy + 0x7FFFFFFF) & 0xFFFFFFFF);
        cells[key] = p;
    }

    cloud_.clear();
    cloud_.reserve(cells.size());
    for (const auto& kv : cells)
        cloud_.push_back(kv.second);

    std::cout << "[MapController] Cloud voxel-filtered → " << cloud_.size() << " points\n";
}

Pose2D MapController::getPose() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return pose_;
}
std::vector<Pose2D> MapController::getPath() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return {path_.begin(), path_.end()};
}
std::vector<Eigen::Vector3d> MapController::getCloud() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return cloud_;
}
OccGrid MapController::getGrid() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return grid_;
}

std::vector<Eigen::Vector2d>
MapController::toEigen2D(const std::vector<LidarPoint>& scan) const
{
    std::vector<Eigen::Vector2d> out;
    out.reserve(scan.size());
    for (const auto& p : scan) {
        if (!std::isfinite(p.angle) || !std::isfinite(p.range)) continue;
        if (p.range < 0.15f || p.range > (float)maxRange_)       continue;
        double rad = p.angle * M_PI / 180.0;
        double x   =  std::cos(rad) * p.range;
        double y   = -std::sin(rad) * p.range;
        if (!std::isfinite(x) || !std::isfinite(y))               continue;
        out.emplace_back(x, y);
    }
    return out;
}

std::vector<Eigen::Vector2d>
MapController::transform2D(const std::vector<Eigen::Vector2d>& pts,
                            double tx, double ty, double yaw)
{
    double c = std::cos(yaw), s = std::sin(yaw);
    std::vector<Eigen::Vector2d> out;
    out.reserve(pts.size());
    for (const auto& p : pts)
        out.emplace_back(c * p.x() - s * p.y() + tx,
                         s * p.x() + c * p.y() + ty);
    return out;
}

bool MapController::icp2D(const std::vector<Eigen::Vector2d>& src,
                           const std::vector<Eigen::Vector2d>& ref,
                           double& outDx, double& outDy, double& outDYaw,
                           int maxIter, double tolerance) const
{
    if (src.size() < 20 || ref.size() < 20) return false;

    auto downsample = [](const std::vector<Eigen::Vector2d>& in, size_t maxPts)
        -> std::vector<Eigen::Vector2d>
    {
        if (in.size() <= maxPts) return in;
        std::vector<Eigen::Vector2d> out;
        out.reserve(maxPts);
        size_t step = in.size() / maxPts;
        for (size_t i = 0; i < in.size(); i += step) out.push_back(in[i]);
        return out;
    };

    auto s = downsample(src, 200);
    auto r = downsample(ref, 200);

    double tx = 0, ty = 0, yaw = 0;

    for (int iter = 0; iter < maxIter; ++iter) {
        auto moved = transform2D(s, tx, ty, yaw);

        std::vector<Eigen::Vector2d> srcPts, dstPts;
        srcPts.reserve(moved.size());
        dstPts.reserve(moved.size());

        const double maxDistSq = 0.5 * 0.5;

        for (const auto& mp : moved) {
            double bestDist = std::numeric_limits<double>::max();
            const Eigen::Vector2d* bestRef = nullptr;
            for (const auto& rp : r) {
                double d = (mp - rp).squaredNorm();
                if (d < bestDist) { bestDist = d; bestRef = &rp; }
            }
            if (bestRef && bestDist < maxDistSq) {
                srcPts.push_back(mp);
                dstPts.push_back(*bestRef);
            }
        }

        if (srcPts.size() < 10) return false;

        Eigen::Vector2d srcMean = Eigen::Vector2d::Zero();
        Eigen::Vector2d dstMean = Eigen::Vector2d::Zero();
        for (size_t i = 0; i < srcPts.size(); ++i) {
            srcMean += srcPts[i];
            dstMean += dstPts[i];
        }
        srcMean /= (double)srcPts.size();
        dstMean /= (double)dstPts.size();

        Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
        for (size_t i = 0; i < srcPts.size(); ++i)
            H += (srcPts[i] - srcMean) * (dstPts[i] - dstMean).transpose();

        Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();

        if (R.determinant() < 0) {
            Eigen::Matrix2d V = svd.matrixV();
            V.col(1) *= -1;
            R = V * svd.matrixU().transpose();
        }

        double dAngle = std::atan2(R(1, 0), R(0, 0));
        Eigen::Vector2d dT = dstMean - R * srcMean;

        double c = std::cos(yaw), s2 = std::sin(yaw);
        tx  += c * dT.x() - s2 * dT.y();
        ty  += s2 * dT.x() + c * dT.y();
        yaw += dAngle;

        if (std::abs(dT.x()) < tolerance &&
            std::abs(dT.y()) < tolerance &&
            std::abs(dAngle) < tolerance)
            break;
    }

    if (std::abs(tx)  > 1.0 || std::abs(ty)  > 1.0 ||
        std::abs(yaw) > M_PI / 4.0) {
        std::cerr << "[MapController] ICP result rejected: too large ("
                  << tx << ", " << ty << ", " << yaw << ")\n";
        return false;
    }

    outDx   = tx;
    outDy   = ty;
    outDYaw = yaw;
    return true;
}

void MapController::ensureGridCovers(double wx, double wy)
{
    const double MAX_EXTENT = 20.0;
    if (std::abs(wx) > MAX_EXTENT || std::abs(wy) > MAX_EXTENT) return;

    double x0 = grid_.originX, y0 = grid_.originY;
    double x1 = x0 + grid_.width  * grid_.res;
    double y1 = y0 + grid_.height * grid_.res;

    const double pad = 5.0;
    bool grow = false;

    if (wx < x0 + pad) { grid_.originX -= pad; grid_.width  += (int)(pad / grid_.res); grow = true; }
    if (wy < y0 + pad) { grid_.originY -= pad; grid_.height += (int)(pad / grid_.res); grow = true; }
    if (wx > x1 - pad) { grid_.width   += (int)(pad / grid_.res); grow = true; }
    if (wy > y1 - pad) { grid_.height  += (int)(pad / grid_.res); grow = true; }

    if (grow) {
        grid_.cells.assign(grid_.width * grid_.height, 0);
        std::cout << "[MapController] Grid expanded to "
                  << grid_.width << "×" << grid_.height << " cells\n";
    }
}

bool MapController::worldToCell(double wx, double wy, int& col, int& row) const
{
    col = (int)((wx - grid_.originX) / grid_.res);
    row = (int)((wy - grid_.originY) / grid_.res);
    return col >= 0 && col < grid_.width && row >= 0 && row < grid_.height;
}

void MapController::raycastIntoGrid(const Pose2D& pose,
                                     const std::vector<Eigen::Vector2d>& pts)
{
    ensureGridCovers(pose.x, pose.y);

    int rc, rr;
    if (!worldToCell(pose.x, pose.y, rc, rr)) return;

    double c = std::cos(pose.yaw), s = std::sin(pose.yaw);

    for (const auto& p : pts) {
        double wx = pose.x + c * p.x() - s * p.y();
        double wy = pose.y + s * p.x() + c * p.y();

        ensureGridCovers(wx, wy);

        int ec, er;
        if (!worldToCell(wx, wy, ec, er)) continue;

        int x0 = rc, y0 = rr, x1 = ec, y1 = er;
        int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        while (true) {
            if (x0 == x1 && y0 == y1) {
                int idx = y0 * grid_.width + x0;
                if (idx >= 0 && idx < (int)grid_.cells.size())
                    grid_.cells[idx] = 2;
                break;
            }
            int idx = y0 * grid_.width + x0;
            if (idx >= 0 && idx < (int)grid_.cells.size() && grid_.cells[idx] != 2)
                grid_.cells[idx] = 1;

            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 <  dx) { err += dx; y0 += sy; }
        }
    }
}