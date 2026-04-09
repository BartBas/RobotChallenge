#include "CamController.h"
#include <algorithm>
#include <cmath>
#include <iostream>

using namespace cv;
using namespace std;

/**
 * @brief Constructs a CamController and pre-allocates internal frame buffers.
 *
 * @details
 * Sets all tuneable parameters to their defaults and registers the built-in
 * `"red"` colour (hue bands 0–10 and 170–180) so the controller is usable
 * immediately after construction without any further colour configuration.
 *
 * The combined mask and temporary mask are allocated once here to avoid
 * per-frame heap allocations inside the hot capture path.
 *
 * @param cameraIndex  V4L2 / OpenCV device index (ignored when rpicam-vid is
 *                     used). Default: @c 0.
 * @param width        Desired capture width in pixels. Default: @c 640.
 * @param height       Desired capture height in pixels. Default: @c 480.
 */
 
CamController::CamController(int cameraIndex, int width, int height)
    : cameraIndex(cameraIndex)
    , frameWidth(width)
    , frameHeight(height)
    , minArea(500.0)
    , deadOnThreshold(5.0)
    , strategy(TrackingStrategy::LARGEST)
    , cameraStream(nullptr)
    , useRpiCam(false)
    , streamingEnabled(false)
    , streamPort(8080)
    , flip180(false)
    , collectXMin_(0.55f)
    , collectXMax_(0.75f)
{
    // Pre-allocate masks once — reused every frame, no heap churn
    combinedMask_.create(height, width, CV_8UC1);
    tmpMask_.create(height, width, CV_8UC1);

    // Default red colour
    addTrackedColor("red",
                    0, 10, 100, 100,
                    170, 180, 100, 100);
}

/**
 * @brief Destructor — stops streaming and releases the camera resource.
 *
 * @details
 * Calls `enableStreaming(false)` to signal the stream thread and join it,
 * then calls `release()` to close the rpicam-vid pipe or the OpenCV capture.
 */
CamController::~CamController() {
    enableStreaming(false);
    release();
}

/**
 * @brief Opens the camera and starts the rpicam-vid capture pipeline.
 *
 * @details
 * Launches `rpicam-vid` as a child process via `popen()`, requesting raw
 * YUV 4:2:0 output on stdout at the resolution configured in the constructor.
 * If the pipe cannot be opened the function prints an error and returns
 * @c false; otherwise it sets the `useRpiCam` flag and returns @c true.
 *
 * @note This method is specific to Raspberry Pi. When porting to a platform
 *       without `rpicam-vid`, replace the body with an `cv::VideoCapture`
 *       open call and clear the `useRpiCam` flag.
 *
 * @return @c true if rpicam-vid started successfully, @c false otherwise.
 */
bool CamController::initialize() {
    cout << "Initializing camera using rpicam-vid..." << endl;

    string cmd = "rpicam-vid --width " + to_string(frameWidth) +
                 " --height " + to_string(frameHeight) +
                 " --codec yuv420 --framerate 30 -t 0 -o - 2>/dev/null";

    cameraStream = popen(cmd.c_str(), "r");
    if (!cameraStream) { cerr << "Failed to start rpicam-vid\n"; return false; }

    useRpiCam = true;
    cout << "Camera initialized successfully\n";
    return true;
}

/**
 * @brief Enables or disables 180° frame rotation.
 * @param flip @c true to rotate every captured frame by 180°.
 */
void CamController::setFlip(bool flip) { flip180 = flip; }

/**
 * @brief Returns whether 180° frame rotation is active.
 * @return @c true if frames are currently flipped.
 */
bool CamController::isFlipped() const  { return flip180; }

/**
 * @brief Returns @c true if the camera (or rpicam-vid pipe) is open.
 * @return @c true when a valid capture source is available.
 */
bool CamController::isOpened() const {
    return useRpiCam ? cameraStream != nullptr : cap.isOpened();
}

/**
 * @brief Releases the active camera resource.
 *
 * @details
 * Closes the `rpicam-vid` pipe with `pclose()` when running in RPi mode,
 * or releases the OpenCV `VideoCapture` handle otherwise.
 */
void CamController::release() {
    if (useRpiCam && cameraStream) { pclose(cameraStream); cameraStream = nullptr; }
    else if (cap.isOpened()) cap.release();
}

/**
 * @brief Returns a clone of the most recently captured raw frame.
 * @return A deep copy of `currentFrame` (BGR, unmodified).
 */
cv::Mat CamController::getFrame() const { return currentFrame.clone(); }

/**
 * @brief Returns a clone of the most recently captured frame with all
 *        detection overlays rendered on top.
 *
 * @details
 * Clones `currentFrame`, then calls `drawVisualization()` on the copy so
 * the stored frame is never modified. Acquires `frameMutex_` indirectly
 * through the clone operation — do not call simultaneously with
 * `captureFrame()` from another thread.
 *
 * @return BGR frame with bounding boxes, labels, guide lines, and
 *         centre-to-target line drawn on it.
 */
cv::Mat CamController::getFrameWithVisualization() {
    cv::Mat vis = currentFrame.clone();
    drawVisualization(vis);
    return vis;
}

/**
 * @brief Sets the horizontal collection zone used for stream guide lines and
 *        alignment feedback.
 *
 * @details
 * Both values are normalised (0.0 = left edge, 1.0 = right edge).  The
 * `Brain` uses this same window to decide when the robot is aligned enough
 * to begin COLLECT Phase A.
 *
 * @param xMin Normalised left boundary of the collection zone.
 * @param xMax Normalised right boundary of the collection zone.
 */
void CamController::setCollectionZone(float xMin, float xMax) {
    collectXMin_ = xMin;
    collectXMax_ = xMax;
}

/**
 * @brief Registers a new named colour for tracking.
 *
 * @details
 * Each colour is described by two HSV bands — a lower-hue band and an
 * upper-hue band — which are OR-combined into the frame mask each cycle.
 * This two-band design naturally handles red, which wraps around the HSV
 * hue circle.
 *
 * If @p name already exists its definition is updated in-place (no slot
 * is consumed).  If the table is full and @p name is new the function
 * prints an error and returns @c false.
 *
 * @param name      Unique identifier for the colour (e.g. `"orange"`).
 * @param lHueMin   Lower band: minimum hue value [0, 179].
 * @param lHueMax   Lower band: maximum hue value [0, 179].
 * @param lSatMin   Lower band: minimum saturation [0, 255].
 * @param lValMin   Lower band: minimum value (brightness) [0, 255].
 * @param uHueMin   Upper band: minimum hue value [0, 179].
 * @param uHueMax   Upper band: maximum hue value [0, 179].
 * @param uSatMin   Upper band: minimum saturation [0, 255].
 * @param uValMin   Upper band: minimum value (brightness) [0, 255].
 *
 * @return @c true if the colour was added or updated successfully,
 *         @c false if `MAX_TRACKED_COLORS` would be exceeded.
 */
bool CamController::addTrackedColor(const std::string& name,
                                     int lHueMin, int lHueMax, int lSatMin, int lValMin,
                                     int uHueMin, int uHueMax, int uSatMin, int uValMin)
{
    if (trackedColors.size() >= MAX_TRACKED_COLORS &&
        trackedColors.find(name) == trackedColors.end()) {
        cerr << "Max tracked colors reached, cannot add '" << name << "'\n";
        return false;
    }
    ColorRange cr;
    cr.name     = name;
    cr.lowerMin = cv::Scalar(lHueMin, lSatMin, lValMin);
    cr.lowerMax = cv::Scalar(lHueMax, 255,     255    );
    cr.upperMin = cv::Scalar(uHueMin, uSatMin, uValMin);
    cr.upperMax = cv::Scalar(uHueMax, 255,     255    );
    trackedColors[name] = cr;
    cout << "Added tracked color: " << name << "\n";
    return true;
}

/**
 * @brief Removes a previously registered colour from tracking.
 *
 * @param name Name of the colour to remove.
 * @return @c true if the colour was found and removed, @c false if it
 *         did not exist.
 */
bool CamController::removeTrackedColor(const std::string& name) {
    auto it = trackedColors.find(name);
    if (it == trackedColors.end()) { cerr << "Color '" << name << "' not found\n"; return false; }
    trackedColors.erase(it);
    cout << "Removed tracked color: " << name << "\n";
    return true;
}

/**
 * @brief Returns @c true if a colour with the given name is currently tracked.
 * @param name Colour identifier to look up.
 * @return @c true if @p name is in the tracking table.
 */
bool CamController::hasColor(const std::string& name) const {
    return trackedColors.count(name) > 0;
}

/**
 * @brief Returns the number of colours currently registered for tracking.
 * @return Integer in the range [0, MAX_TRACKED_COLORS].
 */
int CamController::getColorCount() const { return (int)trackedColors.size(); }

/**
 * @brief Returns the names of all currently tracked colours.
 * @return Vector of colour name strings in map iteration order.
 */
std::vector<std::string> CamController::getColorNames() const {
    std::vector<std::string> v;
    v.reserve(trackedColors.size());
    for (const auto& p : trackedColors) v.push_back(p.first);
    return v;
}

/**
 * @brief Removes every registered colour from the tracking table.
 *
 * @note After calling this, no objects will be detected until at least one
 *       colour is re-registered with `addTrackedColor()`.
 */
void CamController::clearAllColors() {
    trackedColors.clear();
    cout << "Cleared all tracked colors.\n";
}

/**
 * @brief Sets the lower HSV band for the built-in `"red"` colour entry.
 *
 * @details
 * Legacy compatibility wrapper. Has no effect if `"red"` has been removed
 * from the tracking table.
 *
 * @param hMin Minimum hue  [0, 179].
 * @param hMax Maximum hue  [0, 179].
 * @param sMin Minimum saturation [0, 255]. Default: 100.
 * @param vMin Minimum value (brightness) [0, 255]. Default: 100.
 */
void CamController::setRedRangeLower(int hMin, int hMax, int sMin, int vMin) {
    if (!hasColor("red")) return;
    trackedColors["red"].lowerMin = Scalar(hMin, sMin, vMin);
    trackedColors["red"].lowerMax = Scalar(hMax, 255, 255);
}

/**
 * @brief Sets the upper HSV band for the built-in `"red"` colour entry.
 *
 * @details
 * Legacy compatibility wrapper. Has no effect if `"red"` has been removed
 * from the tracking table.
 *
 * @param hMin Minimum hue  [0, 179].
 * @param hMax Maximum hue  [0, 179].
 * @param sMin Minimum saturation [0, 255]. Default: 100.
 * @param vMin Minimum value (brightness) [0, 255]. Default: 100.
 */
void CamController::setRedRangeUpper(int hMin, int hMax, int sMin, int vMin) {
    if (!hasColor("red")) return;
    trackedColors["red"].upperMin = Scalar(hMin, sMin, vMin);
    trackedColors["red"].upperMax = Scalar(hMax, 255, 255);
}

/**
 * @brief Captures one frame from the camera and runs the full detection
 *        pipeline on it.
 *
 * @details
 * **RPi path:** reads exactly one YUV 4:2:0 frame from the rpicam-vid
 * pipe, converts it to BGR, and optionally flips it 180°. The frame is
 * stored under `frameMutex_` so the streaming thread can access it safely.
 *
 * **Generic path:** grabs from the OpenCV `VideoCapture` and flips if
 * requested.
 *
 * After storing the frame, `detectColorPixels()` and `findColorObjects()`
 * are called to populate `detectedObjects_` for the current cycle.
 *
 * @return @c true if a frame was successfully read, @c false on stream
 *         error or empty frame.
 */
bool CamController::captureFrame()
{
    if (useRpiCam) {
        int frameSize = frameWidth * frameHeight * 3 / 2;   // YUV420
        std::vector<uint8_t> buf(frameSize);

        size_t total = 0;
        while (total < (size_t)frameSize) {
            size_t n = fread(buf.data() + total, 1, frameSize - total, cameraStream);
            if (n == 0) { cerr << "Stream ended or error\n"; return false; }
            total += n;
        }

        {
            std::lock_guard<std::mutex> lk(frameMutex);
            cv::Mat yuv(frameHeight * 3 / 2, frameWidth, CV_8UC1, buf.data());
            cv::cvtColor(yuv, currentFrame, cv::COLOR_YUV2BGR_I420);
            if (flip180) cv::flip(currentFrame, currentFrame, -1);
        }

    } else {
        cap >> currentFrame;
        if (currentFrame.empty()) return false;
        if (flip180) cv::flip(currentFrame, currentFrame, -1);
    }

    detectColorPixels();
    detectedObjects = findColorObjects();
    return true;
}

/**
 * @brief Converts the current BGR frame to HSV and builds a combined colour
 *        mask for all registered colours.
 *
 * @details
 * Performs a single `cv::cvtColor` (BGR → HSV), then iterates over every
 * registered `ColorRange`. For each colour both the lower and upper HSV
 * bands are thresholded with `cv::inRange` into `tmpMask_`, which is then
 * OR-merged into `combinedMask_`. Using pre-allocated member mats avoids
 * heap allocations on the hot path.
 *
 * Called internally by `captureFrame()`.
 */
void CamController::detectColorPixels()
{
    cv::cvtColor(currentFrame, hsvFrame_, cv::COLOR_BGR2HSV);
    combinedMask_.setTo(0);

    for (const auto& kv : trackedColors) {
        const ColorRange& cr = kv.second;
        cv::inRange(hsvFrame_, cr.lowerMin, cr.lowerMax, tmpMask_);
        cv::bitwise_or(combinedMask_, tmpMask_, combinedMask_);
        cv::inRange(hsvFrame_, cr.upperMin, cr.upperMax, tmpMask_);
        cv::bitwise_or(combinedMask_, tmpMask_, combinedMask_);
    }
}

/**
 * @brief Finds and filters colour blobs in the current combined mask.
 *
 * @details
 * Runs `cv::findContours` once on `combinedMask_` (RETR_EXTERNAL, no mask
 * copy needed). For each contour whose area exceeds `minArea_`:
 *
 * 1. **Centroid** is computed from image moments (falls back to bounding-box
 *    centre when the moment is zero).
 * 2. **Elevated-object filter** rejects any blob that is both large
 *    (area > `elevatedAreaThresh_`) and sits in the top portion of the frame
 *    (normalised Y < `elevatedYThresh_`). With the camera ~7 cm off the
 *    ground, real cups always appear low in the frame; large blobs near the
 *    top are assumed to be elevated furniture.
 * 3. **Colour identification** samples the HSV pixel at the centroid and
 *    matches it against each registered `ColorRange`.  The first match wins;
 *    unmatched blobs are labelled `"unknown"`.
 *
 * Called internally by `captureFrame()`.
 *
 * @return Vector of `RedObject` structs, one per accepted blob.
 */
std::vector<CamController::RedObject> CamController::findColorObjects()
{
    std::vector<RedObject> objects;
    std::vector<std::vector<cv::Point>> contours;

    // RETR_EXTERNAL does not modify the source mat so no .clone() needed
    cv::findContours(combinedMask_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    objects.reserve(contours.size());

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < minArea) continue;

        RedObject obj;
        obj.area        = area;
        obj.boundingBox = cv::boundingRect(contour);

        cv::Moments m = cv::moments(contour);
        obj.center = (m.m00 > 0)
            ? cv::Point2f((float)(m.m10 / m.m00), (float)(m.m01 / m.m00))
            : cv::Point2f(obj.boundingBox.x + obj.boundingBox.width  * 0.5f,
                          obj.boundingBox.y + obj.boundingBox.height * 0.5f);

        // ── Elevated-object filter ────────────────────────────────────────
        // The camera sits only 7 cm off the ground, so real cups always appear
        // in the lower portion of the frame regardless of distance.  A large
        // object whose centre is in the top 40 % of the frame (normalised Y
        // < 0.40) is almost certainly elevated furniture (chair back, table
        // leg, etc.) — reject it.
        //
        // The threshold scales with area: small blobs in the upper frame are
        // tolerated (might be a distant cup near the horizon), but anything
        // large AND high is rejected outright.
        //
        //   norm_y = 0   → top of frame   (suspicious if large)
        //   norm_y = 1   → bottom of frame (always fine)
        //
        // Rejection condition:
        //   area > elevatedAreaThresh  AND  normY < elevatedYThresh
        {
            float normY = (frameHeight > 0)
                          ? (obj.center.y / (float)frameHeight)
                          : 1.0f;

            if (area > elevatedAreaThresh_ && normY < elevatedYThresh_) {
                // Silently skip — large blob sitting high in frame
                continue;
            }
        }

        // Identify colour: check HSV pixel at centroid against each range
        obj.colorName = "unknown";
        if (!hsvFrame_.empty()) {
            int cx = std::clamp((int)obj.center.x, 0, hsvFrame_.cols - 1);
            int cy = std::clamp((int)obj.center.y, 0, hsvFrame_.rows - 1);
            cv::Vec3b px = hsvFrame_.at<cv::Vec3b>(cy, cx);

            for (const auto& kv : trackedColors) {
                const ColorRange& cr = kv.second;
                auto hit = [&](const cv::Scalar& lo, const cv::Scalar& hi) {
                    return px[0] >= lo[0] && px[0] <= hi[0]
                        && px[1] >= lo[1] && px[1] <= hi[1]
                        && px[2] >= lo[2] && px[2] <= hi[2];
                };
                if (hit(cr.lowerMin, cr.lowerMax) || hit(cr.upperMin, cr.upperMax)) {
                    obj.colorName = kv.first;
                    break;
                }
            }
        }

        objects.push_back(std::move(obj));
    }

    return objects;
}

/**
 * @brief Public entry point — returns the tracking direction for the current
 *        frame.
 *
 * @details Delegates to `analyzeColorObjects()`.
 *
 * @return `Direction` struct with bearing angle, human-readable command
 *         string, normalised distance, and total object count.
 */
CamController::Direction CamController::getDirection() {
    return analyzeColorObjects();
}

/**
 * @brief Selects the primary target according to the active
 *        `TrackingStrategy` and computes the resulting `Direction`.
 *
 * @details
 * If no objects are detected, returns a `Direction` with `command =
 * "NO TARGET"` and zeroed angle / distance fields.
 *
 * Otherwise the primary target is chosen as follows:
 * - **LARGEST** — object with the greatest contour area.
 * - **CLOSEST_TO_CENTER** — object whose centre X is nearest the frame
 *   centre.
 * - **LEFTMOST** — object with the smallest centre X coordinate.
 * - **LOWEST** — object with the largest centre Y coordinate (closest to
 *   the bottom of the frame, i.e. nearest the robot).
 *
 * The bearing angle is mapped linearly from the target's horizontal
 * position: centre → 0°, full right → +90°, full left → −90°.  When
 * `|angle| ≤ deadOnThreshold` the command is `"FORWARD"`, otherwise it
 * is `"LEFT N degrees"` or `"RIGHT N degrees"`, both suffixed with the
 * matched colour name.
 *
 * @return Populated `Direction` struct.
 */
CamController::Direction CamController::analyzeColorObjects()
{
    Direction dir;
    dir.objectCount = (int)detectedObjects.size();

    if (detectedObjects.empty()) {
        dir.angle = 0; dir.distance = 0; dir.command = "NO TARGET";
        return dir;
    }

    const RedObject* tgt = nullptr;

    if (strategy == TrackingStrategy::LARGEST) {
        tgt = &(*std::max_element(detectedObjects.begin(), detectedObjects.end(),
            [](const RedObject& a, const RedObject& b){ return a.area < b.area; }));
    }
    else if (strategy == TrackingStrategy::CLOSEST_TO_CENTER) {
        float cx = frameWidth * 0.5f;
        tgt = &(*std::min_element(detectedObjects.begin(), detectedObjects.end(),
            [cx](const RedObject& a, const RedObject& b){
                return std::abs(a.center.x - cx) < std::abs(b.center.x - cx);
            }));
    }
    else if (strategy == TrackingStrategy::LEFTMOST) {
        tgt = &(*std::min_element(detectedObjects.begin(), detectedObjects.end(),
            [](const RedObject& a, const RedObject& b){ return a.center.x < b.center.x; }));
    }
    else if (strategy == TrackingStrategy::LOWEST) {
        tgt = &(*std::max_element(detectedObjects.begin(), detectedObjects.end(),
            [](const RedObject& a, const RedObject& b){ return a.center.y < b.center.y; }));
    }

    if (!tgt) { dir.angle = 0; dir.distance = 0; dir.command = "NO TARGET"; return dir; }

    double normX  = (tgt->center.x - frameWidth * 0.5) / (frameWidth * 0.5);
    dir.angle    = normX * 90.0;
    dir.distance = normX;

    if (std::abs(dir.angle) <= deadOnThreshold)
        dir.command = "FORWARD (" + tgt->colorName + ")";
    else if (dir.angle > 0)
        dir.command = "RIGHT " + std::to_string((int)std::abs(dir.angle)) + " degrees (" + tgt->colorName + ")";
    else
        dir.command = "LEFT "  + std::to_string((int)std::abs(dir.angle)) + " degrees (" + tgt->colorName + ")";

    return dir;
}

/**
 * @brief Returns a copy of the objects detected during the last
 *        `captureFrame()` call.
 * @return Vector of `RedObject` structs (may be empty if none detected).
 */
std::vector<CamController::RedObject> CamController::getDetectedObjects() const {
    return detectedObjects;
}

/**
 * @brief Sets the active tracking strategy.
 * @param s One of `LARGEST`, `CLOSEST_TO_CENTER`, `LEFTMOST`, or `LOWEST`.
 */
void   CamController::setTrackingStrategy(TrackingStrategy s) { strategy = s; }

/**
 * @brief Returns the currently active tracking strategy.
 * @return The current `TrackingStrategy` enum value.
 */
CamController::TrackingStrategy CamController::getTrackingStrategy() const { return strategy; }

/**
 * @brief Sets the minimum contour area (px²) for a blob to be accepted.
 *
 * @details Blobs smaller than this value are silently discarded in
 *          `findColorObjects()`. Increase to reduce noise; decrease to
 *          detect smaller or more distant objects.
 *
 * @param a Minimum area in pixels squared. Default: @c 500.0.
 */
void   CamController::setMinArea(double a)          { minArea = a; }

/**
 * @brief Returns the current minimum contour area threshold.
 * @return Minimum area in pixels squared.
 */
double CamController::getMinArea() const            { return minArea; }

/**
 * @brief Sets the angular dead-band within which the target is considered
 *        centred.
 *
 * @details When `|bearing angle| ≤ threshold` the command is reported as
 *          `"FORWARD"` rather than a directional turn. Measured in degrees.
 *
 * @param t Dead-on threshold in degrees. Default: @c 5.0.
 */
void   CamController::setDeadOnThreshold(double t)  { deadOnThreshold = t; }

/**
 * @brief Returns the current dead-on angular threshold.
 * @return Threshold in degrees.
 */
double CamController::getDeadOnThreshold() const    { return deadOnThreshold; }

/**
 * @brief Renders all detection overlays onto @p frame in-place.
 *
 * @details Draws the following elements:
 * - **Collection zone guide lines** — two vertical purple lines at
 *   `collectXMin_` and `collectXMax_` (normalised X), labelled
 *   `"COLLECT ZONE"`.
 * - **Per-object markers** — a circle at the centroid and a bounding
 *   rectangle for every detected object.  The primary target (selected
 *   by the current strategy) is drawn in green; all others in grey.
 *   Each is labelled with its colour name, index, and `"(TARGET)"` tag
 *   where applicable.
 * - **Frame-centre dot** — blue filled circle at (cols/2, rows/2).
 * - **Target line** — yellow line from the frame centre to the primary
 *   target centroid.
 * - **Alignment highlight** — when the primary target's normalised X
 *   falls within the collection zone a semi-transparent green rectangle
 *   is overlaid and `"ALIGNED"` is printed.
 *
 * @param frame BGR image to draw on (modified in-place).
 */
void CamController::drawVisualization(cv::Mat& frame)
{
    // ── Collection zone guide lines ───────────────────────────────────────
    {
        const cv::Scalar purple(200, 0, 200);
        int xL = (int)(collectXMin_ * frame.cols);
        int xR = (int)(collectXMax_ * frame.cols);
        cv::line(frame, {xL, 0}, {xL, frame.rows - 1}, purple, 2);
        cv::line(frame, {xR, 0}, {xR, frame.rows - 1}, purple, 2);
        cv::putText(frame, "COLLECT ZONE", {xL + 4, 18},
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, purple, 1);
    }

    if (detectedObjects.empty()) return;

    // ── Pick target ───────────────────────────────────────────────────────
    const RedObject* tgt = nullptr;
    if (strategy == TrackingStrategy::LARGEST) {
        tgt = &(*std::max_element(detectedObjects.begin(), detectedObjects.end(),
            [](const RedObject& a, const RedObject& b){ return a.area < b.area; }));
    }
    else if (strategy == TrackingStrategy::CLOSEST_TO_CENTER) {
        float cx = frame.cols * 0.5f;
        tgt = &(*std::min_element(detectedObjects.begin(), detectedObjects.end(),
            [cx](const RedObject& a, const RedObject& b){
                return std::abs(a.center.x - cx) < std::abs(b.center.x - cx);
            }));
    }
    else if (strategy == TrackingStrategy::LEFTMOST) {
        tgt = &(*std::min_element(detectedObjects.begin(), detectedObjects.end(),
            [](const RedObject& a, const RedObject& b){ return a.center.x < b.center.x; }));
    }
    else if (strategy == TrackingStrategy::LOWEST) {
        tgt = &(*std::max_element(detectedObjects.begin(), detectedObjects.end(),
            [](const RedObject& a, const RedObject& b){ return a.center.y < b.center.y; }));
    }

    // ── Draw objects ──────────────────────────────────────────────────────
    for (size_t i = 0; i < detectedObjects.size(); ++i) {
        bool isT = (&detectedObjects[i] == tgt);
        cv::Scalar col = isT ? cv::Scalar(0, 255, 0) : cv::Scalar(128, 128, 128);
        cv::circle(frame, detectedObjects[i].center, 10, col, 2);
        cv::rectangle(frame, detectedObjects[i].boundingBox, col, 2);
        std::string lbl = detectedObjects[i].colorName + " " + std::to_string(i + 1);
        if (isT) lbl += " (TARGET)";
        cv::putText(frame, lbl,
                    {detectedObjects[i].boundingBox.x,
                     detectedObjects[i].boundingBox.y - 10},
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, col, 2);
    }

    // ── Centre dot + line to target ───────────────────────────────────────
    cv::Point fc(frame.cols / 2, frame.rows / 2);
    cv::circle(frame, fc, 5, cv::Scalar(255, 0, 0), -1);

    if (tgt) {
        cv::line(frame, fc, tgt->center, cv::Scalar(255, 255, 0), 2);

        float normX = tgt->center.x / (float)frame.cols;
        if (normX >= collectXMin_ && normX <= collectXMax_) {
            int xL = (int)(collectXMin_ * frame.cols);
            int xR = (int)(collectXMax_ * frame.cols);
            cv::Mat overlay = frame.clone();
            cv::rectangle(overlay, {xL, 0}, {xR, frame.rows - 1},
                          cv::Scalar(0, 200, 0), cv::FILLED);
            cv::addWeighted(overlay, 0.15, frame, 0.85, 0, frame);
            cv::putText(frame, "ALIGNED", {xL + 4, 36},
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        }
    }
}

/**
 * @brief Enables or disables the background MJPEG streaming server.
 *
 * @details
 * **Enabling:** stores the port, sets `streamingEnabled` to @c true, and
 * launches a detached `std::thread` running `streamingLoop()`.  A message
 * with the stream URL is printed to stdout.
 *
 * **Disabling:** sets `streamingEnabled` to @c false (which causes
 * `streamingLoop()` to exit its inner loops on the next iteration) and
 * joins the thread.
 *
 * Calling `enableStreaming(true)` while already streaming, or
 * `enableStreaming(false)` while not streaming, is a no-op.
 *
 * @param enable @c true to start streaming, @c false to stop.
 * @param port   TCP port the server will listen on. Default: @c 8080.
 */
void CamController::enableStreaming(bool enable, int port) {
    if (enable && !streamingEnabled) {
        streamPort = port; streamingEnabled = true;
        streamThread = std::thread(&CamController::streamingLoop, this);
        cout << "Streaming enabled on http://<your-pi-ip>:" << port << "\n";
    } else if (!enable && streamingEnabled) {
        streamingEnabled = false;
        if (streamThread.joinable()) streamThread.join();
        cout << "Streaming disabled\n";
    }
}

/**
 * @brief Returns whether the MJPEG streaming server is currently running.
 * @return @c true if the stream thread is active.
 */
bool CamController::isStreamingEnabled() const { return streamingEnabled; }

/**
 * @brief Returns the TCP port the MJPEG server is (or will be) listening on.
 * @return Port number.
 */
int  CamController::getStreamPort() const { return streamPort; }

/**
 * @brief Background thread body for the single-client MJPEG streaming server.
 *
 * @details
 * Creates a TCP socket, binds to `streamPort`, and enters an accept loop.
 * For each connected client:
 * 1. Sends the HTTP multipart response header.
 * 2. Enters a frame loop that grabs the latest visualised frame (under
 *    `frameMutex`), JPEG-encodes it at quality 80, wraps it in a MIME
 *    boundary part, and sends it.
 * 3. Sleeps ~66 ms between frames to target ~15 FPS.
 * 4. Exits the frame loop and closes the client socket when `send()`
 *    fails (client disconnected) or `streamingEnabled` becomes @c false.
 *
 * Only one client is served at a time (`listen(srv, 1)`). The server
 * socket is closed when `streamingEnabled` becomes @c false and the
 * accept loop exits.
 *
 * @note Called exclusively by the thread launched in `enableStreaming()`.
 *       Do not call directly.
 */
void CamController::streamingLoop()
{
    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) { cerr << "Failed to create socket\n"; return; }

    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(streamPort);

    if (bind(srv, (sockaddr*)&addr, sizeof(addr)) < 0) {
        cerr << "Failed to bind to port " << streamPort << "\n";
        close(srv); return;
    }

    listen(srv, 1);
    cout << "Stream server listening on port " << streamPort << "\n";

    while (streamingEnabled) {
        sockaddr_in cli{}; socklen_t cliLen = sizeof(cli);
        int fd = accept(srv, (sockaddr*)&cli, &cliLen);
        if (fd < 0) continue;
        cout << "Client connected to stream\n";

        const std::string hdr =
            "HTTP/1.0 200 OK\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
        send(fd, hdr.c_str(), hdr.size(), 0);

        while (streamingEnabled) {
            cv::Mat frame;
            {
                std::lock_guard<std::mutex> lk(frameMutex);
                if (currentFrame.empty()) break;
                frame = getFrameWithVisualization();
            }

            std::vector<uchar> buf;
            cv::imencode(".jpg", frame, buf, {cv::IMWRITE_JPEG_QUALITY, 80});

            std::string part = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                             + std::to_string(buf.size()) + "\r\n\r\n";

            if (send(fd, part.c_str(), part.size(), 0) < 0) break;
            if (send(fd, buf.data(),   buf.size(),   0) < 0) break;
            if (send(fd, "\r\n", 2,                  0) < 0) break;

            usleep(66000);  // ~15 FPS
        }

        close(fd);
        cout << "Client disconnected from stream\n";
    }

    close(srv);
}