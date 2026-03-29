#include "MapController.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <limits>

// ─── constructor ────────────────────────────────────────────────────
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

// ─── reset ──────────────────────────────────────────────────────────
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

// ─── update ─────────────────────────────────────────────────────────
void MapController::update(const std::vector<LidarPoint>& scan)
{
    if (scan.empty()) return;

    auto pts = toEigen2D(scan);
    if (pts.size() < 100) return;   // still spinning up

    // On the first frame we have nothing to match against — just store and wait
    if (!hasPrev_) {
        std::lock_guard<std::mutex> lk(mutex_);
        prevScan_ = pts;
        hasPrev_  = true;
        std::cout << "[MapController] First scan stored, odometry starts next frame\n";
        return;
    }

    // ── Scan-to-scan ICP ──────────────────────────────────────────
    double dx = 0, dy = 0, dYaw = 0;
    bool ok = icp2D(pts, prevScan_, dx, dy, dYaw);

    std::lock_guard<std::mutex> lk(mutex_);

    if (ok) {
        // Compose relative transform with accumulated pose
        double c = std::cos(pose_.yaw);
        double s = std::sin(pose_.yaw);
        pose_.x   += c * dx - s * dy;
        pose_.y   += s * dx + c * dy;
        pose_.yaw += dYaw;
        // Keep yaw in [-π, π]
        while (pose_.yaw >  M_PI) pose_.yaw -= 2.0 * M_PI;
        while (pose_.yaw < -M_PI) pose_.yaw += 2.0 * M_PI;
    }

    // Always update previous scan (even on ICP failure, so we don't get stuck)
    prevScan_ = pts;

    // ── Trail ─────────────────────────────────────────────────────
    path_.push_back(pose_);
    if (path_.size() > maxPath_) path_.pop_front();

    // ── Accumulate world-frame cloud ──────────────────────────────
    double c = std::cos(pose_.yaw);
    double s = std::sin(pose_.yaw);
    for (const auto& p : pts) {
        double wx = pose_.x + c * p.x() - s * p.y();
        double wy = pose_.y + s * p.x() + c * p.y();
        cloud_.emplace_back(wx, wy, 0.0);
    }
    if (cloud_.size() > 1'000'000)
        cloud_.erase(cloud_.begin(), cloud_.begin() + 200'000);

    // ── Occupancy grid ────────────────────────────────────────────
    raycastIntoGrid(pose_, pts);
}

// ─── accessors ──────────────────────────────────────────────────────
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

// ─── toEigen2D ───────────────────────────────────────────────────────
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

// ─── transform2D ────────────────────────────────────────────────────
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

// ─── icp2D ───────────────────────────────────────────────────────────
// Point-to-point 2D ICP using SVD.
// src = current scan, ref = previous scan (both in robot-local frame).
// Outputs the relative transform (dx, dy, dYaw) that aligns src → ref.
bool MapController::icp2D(const std::vector<Eigen::Vector2d>& src,
                           const std::vector<Eigen::Vector2d>& ref,
                           double& outDx, double& outDy, double& outDYaw,
                           int maxIter, double tolerance) const
{
    if (src.size() < 20 || ref.size() < 20) return false;

    // Downsample to at most 200 points each for speed on the Pi
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

    double tx = 0, ty = 0, yaw = 0;   // accumulated transform

    for (int iter = 0; iter < maxIter; ++iter) {
        // Apply current estimate to source points
        auto moved = transform2D(s, tx, ty, yaw);

        // Find nearest neighbour in ref for each moved point (brute force — fast enough at 200 pts)
        std::vector<Eigen::Vector2d> srcPts, dstPts;
        srcPts.reserve(moved.size());
        dstPts.reserve(moved.size());

        const double maxDistSq = 0.5 * 0.5;   // reject correspondences > 50 cm apart

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

        if (srcPts.size() < 10) return false;   // too few correspondences

        // Compute centroids
        Eigen::Vector2d srcMean = Eigen::Vector2d::Zero();
        Eigen::Vector2d dstMean = Eigen::Vector2d::Zero();
        for (size_t i = 0; i < srcPts.size(); ++i) {
            srcMean += srcPts[i];
            dstMean += dstPts[i];
        }
        srcMean /= (double)srcPts.size();
        dstMean /= (double)dstPts.size();

        // Build 2×2 covariance matrix H
        Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
        for (size_t i = 0; i < srcPts.size(); ++i)
            H += (srcPts[i] - srcMean) * (dstPts[i] - dstMean).transpose();

        // SVD → optimal rotation
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();

        // Ensure proper rotation (det = +1)
        if (R.determinant() < 0) {
            Eigen::Matrix2d V = svd.matrixV();
            V.col(1) *= -1;
            R = V * svd.matrixU().transpose();
        }

        double dAngle = std::atan2(R(1, 0), R(0, 0));
        Eigen::Vector2d dT = dstMean - R * srcMean;

        // Accumulate
        double c = std::cos(yaw), s2 = std::sin(yaw);
        tx  += c * dT.x() - s2 * dT.y();
        ty  += s2 * dT.x() + c * dT.y();
        yaw += dAngle;

        // Convergence check
        if (std::abs(dT.x()) < tolerance &&
            std::abs(dT.y()) < tolerance &&
            std::abs(dAngle) < tolerance)
            break;
    }

    // Sanity check — reject implausible motion (> 1 m or > 45° per scan)
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

// ─── ensureGridCovers ───────────────────────────────────────────────
void MapController::ensureGridCovers(double wx, double wy)
{
    double x0 = grid_.originX, y0 = grid_.originY;
    double x1 = x0 + grid_.width  * grid_.res;
    double y1 = y0 + grid_.height * grid_.res;

    const double pad = 5.0;
    bool grow = false;

    if (wx < x0 + pad) { grid_.originX -= pad; grid_.width  += (int)(pad/grid_.res); grow = true; }
    if (wy < y0 + pad) { grid_.originY -= pad; grid_.height += (int)(pad/grid_.res); grow = true; }
    if (wx > x1 - pad) { grid_.width  += (int)(pad/grid_.res); grow = true; }
    if (wy > y1 - pad) { grid_.height += (int)(pad/grid_.res); grow = true; }

    if (grow) {
        grid_.cells.assign(grid_.width * grid_.height, 0);
        std::cout << "[MapController] Grid expanded to "
                  << grid_.width << "×" << grid_.height << " cells\n";
    }
}

// ─── worldToCell ────────────────────────────────────────────────────
bool MapController::worldToCell(double wx, double wy, int& col, int& row) const
{
    col = (int)((wx - grid_.originX) / grid_.res);
    row = (int)((wy - grid_.originY) / grid_.res);
    return col >= 0 && col < grid_.width && row >= 0 && row < grid_.height;
}

// ─── raycastIntoGrid ────────────────────────────────────────────────
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

        // Bresenham ray from robot cell to hit cell
        int x0=rc, y0=rr, x1=ec, y1=er;
        int dx=std::abs(x1-x0), dy=std::abs(y1-y0);
        int sx=(x0<x1)?1:-1, sy=(y0<y1)?1:-1;
        int err=dx-dy;

        while (true) {
            if (x0==x1 && y0==y1) {
                int idx = y0*grid_.width + x0;
                if (idx>=0 && idx<(int)grid_.cells.size())
                    grid_.cells[idx] = 2;   // OCCUPIED
                break;
            }
            int idx = y0*grid_.width + x0;
            if (idx>=0 && idx<(int)grid_.cells.size() && grid_.cells[idx] != 2)
                grid_.cells[idx] = 1;       // FREE

            int e2 = 2*err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 <  dx) { err += dx; y0 += sy; }
        }
    }
}
