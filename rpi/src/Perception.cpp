#include "Perception.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <stdexcept>
#include <iostream>
#include <chrono>

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

static double now_sec()
{
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// ─────────────────────────────────────────────────────────────────────────────
// Constructor / Destructor
// ─────────────────────────────────────────────────────────────────────────────

Perception::Perception(std::string camera_device, bool video_file)
{
    if (video_file) {
        _cap = cv::VideoCapture(camera_device);
    } else {
        _cap = cv::VideoCapture(std::stoi(camera_device));
    }

    if (!_cap.isOpened()) {
        std::cerr << "Failed to open camera\n";
    }

    // Intrinsic matrix  
    // Flatten row-major into a 1D vector
    std::vector<double> flat;
    for (const auto& row : INTRINSIC_MATRIX)
        flat.insert(flat.end(), row.begin(), row.end());

    _camera_matrix = cv::Mat(3, 3, CV_64F, flat.data()).clone();

    // Distortion coefficients (1×N)
    std::vector<double> dist_flat(DISTORTION_COEFFICIENTS.begin(), DISTORTION_COEFFICIENTS.end());
    _dist_coeffs = cv::Mat(1, dist_flat.size(), CV_64F, dist_flat.data()).clone();

    // Mounting height (metres above the floor plane)
    _mounting_height = MOUNTING_HEIGHT;

    // Latest BGR frame
    _latest_bgr_frame = cv::Mat();

    // ── Fixed HSV thresholds ─────────────────────────────────────────────────
    // Part A: "wrap-around" red  (174–179)
    _lower_A = cv::Scalar(174, 150, 100);
    _upper_A = cv::Scalar(179, 255, 255);

    // Part B: "orange" red  (0–14)
    _lower_B = cv::Scalar(0, 150, 100);
    _upper_B = cv::Scalar(10, 255, 255);
}

Perception::~Perception() {}

// ─────────────────────────────────────────────────────────────────────────────
// Step 1 – Red mask
// ─────────────────────────────────────────────────────────────────────────────

cv::Mat Perception::get_red_mask(const cv::Mat& bgr_image) const
{
    cv::Mat blurred;
    cv::GaussianBlur(bgr_image, blurred, cv::Size(5, 5), 0);

    cv::Mat hsv;
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask_A, mask_B;
    cv::inRange(hsv, _lower_A, _upper_A, mask_A);
    cv::inRange(hsv, _lower_B, _upper_B, mask_B);

    cv::Mat mask;
    cv::bitwise_or(mask_A, mask_B, mask);
    return mask;
}

// ─────────────────────────────────────────────────────────────────────────────
// Step 2 – Morphological cleanup
// ─────────────────────────────────────────────────────────────────────────────

cv::Mat Perception::clean_mask(const cv::Mat& mask) const
{
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

    cv::Mat result;
    cv::morphologyEx(mask,   result, cv::MORPH_CLOSE, kernel); // fill gaps
    cv::morphologyEx(result, result, cv::MORPH_OPEN,  kernel); // remove noise
    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// Step 3 – Ridge / medial-axis extraction via distance-transform local maxima
// ─────────────────────────────────────────────────────────────────────────────

cv::Mat Perception::extract_ridge(const cv::Mat& mask, int height_filter) const
{
    // Distance transform (float32)
    cv::Mat dist;
    cv::distanceTransform(mask, dist, cv::DIST_L2, 5);

    // Normalise to uint8 so integer comparison is exact
    cv::Mat dist_norm;
    cv::normalize(dist, dist_norm, 0, 255, cv::NORM_MINMAX, CV_8U);

    // Dilated version – pixels equal to their dilation are local maxima
    cv::Mat dilated;
    cv::dilate(dist_norm, dilated, cv::Mat::ones(3, 3, CV_8U));

    // Local maximum AND minimum distance threshold (≈ half line-width in px)
    cv::Mat local_max = (dist_norm == dilated) & (dist > 5.0f);

    cv::Mat ridge;
    local_max.convertTo(ridge, CV_8U, 255.0);

    // Blank out rows above the height filter (image coordinates: row 0 = top)
    if (height_filter > 0 && height_filter < ridge.rows)
        ridge(cv::Rect(0, 0, ridge.cols, height_filter)).setTo(0);

    return ridge;
}

// ─────────────────────────────────────────────────────────────────────────────
// Step 4 – Collect non-zero pixels as 2-D point list
// ─────────────────────────────────────────────────────────────────────────────

std::vector<cv::Point2f> Perception::extract_points(const cv::Mat& ridge) const
{
    std::vector<cv::Point> pts_i;
    cv::findNonZero(ridge, pts_i);

    std::vector<cv::Point2f> pts;
    pts.reserve(pts_i.size());
    for (const auto& p : pts_i)
        pts.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));

    return pts;
}

// ─────────────────────────────────────────────────────────────────────────────
// Step 5 – Convert 2-D points to 3-D coordinates
// Assumes a vertically mounted camera at _mounting_height above a
// completely horizontal floor.  Coordinate frame: X right, Y down (depth), Z forward.
//
//   depth  = mounting_height
//   Z      = depth · fy / (v – cy)
//   X      = Z · (u – cx) / fx
// ─────────────────────────────────────────────────────────────────────────────

std::vector<cv::Point3d> 
Perception::points2d_to_3d(const std::vector<cv::Point2f>& points_2d) const
{
    if (points_2d.empty()) return {};

    // Undistort pixel coordinates
    std::vector<cv::Point2f> undistorted;
    cv::undistortPoints(points_2d, undistorted,
                        _camera_matrix, _dist_coeffs,
                        cv::noArray(), _camera_matrix);

    const double fx = _camera_matrix.at<double>(0, 0);
    const double fy = _camera_matrix.at<double>(1, 1);
    const double cx = _camera_matrix.at<double>(0, 2);
    const double cy = _camera_matrix.at<double>(1, 2);
    const double depth = _mounting_height;

    std::vector<cv::Point3d> pts3d;
    pts3d.reserve(undistorted.size());

    for (const auto& uv : undistorted)
    {
        double u = uv.x;
        double v = uv.y;

        double Y = depth;
        double Z = Y * fy / (v - cy);
        double X = Z * (u - cx) / fx;

        pts3d.emplace_back(X, Y, Z);
    }

    return pts3d;
}

// ─────────────────────────────────────────────────────────────────────────────
// Public pipeline – detect_line
// ─────────────────────────────────────────────────────────────────────────────

Perception::DetectionResult 
Perception::detect_line(const cv::Mat& bgr_image, int height_filter, bool debug) const
{
    double t0 = now_sec();

    cv::Mat mask  = get_red_mask(bgr_image);          double t1 = now_sec();
    mask          = clean_mask(mask);                  double t2 = now_sec();
    cv::Mat ridge = extract_ridge(mask, height_filter);double t3 = now_sec();
    auto pts2d    = extract_points(ridge);             double t4 = now_sec();
    auto pts3d    = points2d_to_3d(pts2d);             double t5 = now_sec();

    if (debug)
    {
        std::cout << "=======================\n"
                  << "Detection took   " << (t5 - t0) << " s\n"
                  << "Red mask         " << (t1 - t0) << " s\n"
                  << "Clean mask       " << (t2 - t1) << " s\n"
                  << "Extract ridge    " << (t3 - t2) << " s\n"
                  << "Extract points   " << (t4 - t3) << " s\n"
                  << "Points to 3-D    " << (t5 - t4) << " s\n"
                  << "=======================\n";
    }

    return { mask, ridge, pts2d, pts3d };
}


cv::Mat Perception::get_latest_bgr_frame()
{
    std::lock_guard<std::mutex> lock(_mtx);
    _cap.read(_latest_bgr_frame);
    
    return _latest_bgr_frame;
}

std::vector<Position> Perception::get_latest_line_follow_points()
{
    cv::Mat frame;
    {
        std::lock_guard<std::mutex> lock(_mtx);
        if (!_has_frame) return {};
        
        frame = _latest_bgr_frame;
    }

    auto result = detect_line(frame, HEIGHT_FILTER, /*debug=*/false);

    std::vector<Position> positions;
    positions.reserve(result.points_3d.size());
    for (const auto& p : result.points_3d)
        positions.push_back({ p.x, p.y, p.z });

    return positions;
}

// cv::Mat makeDebugView(
//     const cv::Mat& frame_in,
//     const cv::Mat& mask,
//     const cv::Mat& ridge,
//     const std::vector<cv::Point3f>& points_3d,
//     int w, int h)
// {
//     // Resize frame to half resolution
//     cv::Mat frame;
//     cv::resize(frame_in, frame, {w / 2, h / 2}, 0, 0, cv::INTER_NEAREST);
//     w = frame.cols;
//     h = frame.rows;

//     // Normalize single-channel images to BGR for stacking
//     cv::Mat mask_bgr, ridge_bgr;
//     cv::cvtColor(mask,  mask_bgr,  cv::COLOR_GRAY2BGR);
//     cv::cvtColor(ridge, ridge_bgr, cv::COLOR_GRAY2BGR);

//     cv::Mat plot_bgr = renderXYPlot(points_3d,
//         /*xlim=*/{-1000.f, 1000.f},
//         /*ylim=*/{    0.f, 2000.f},
//         /*x_index=*/0,
//         /*y_index=*/2);

//     // Resize all panels to match frame size
//     auto fit = [&](const cv::Mat& img) {
//         cv::Mat out;
//         cv::resize(img, out, {w, h}, 0, 0, cv::INTER_NEAREST);
//         return out;
//     };

//     cv::Mat top, bottom, grid;
//     cv::hconcat(std::vector<cv::Mat>{frame,          fit(mask_bgr)},  top);
//     cv::hconcat(std::vector<cv::Mat>{fit(ridge_bgr), fit(plot_bgr)},  bottom);
//     cv::vconcat(std::vector<cv::Mat>{top, bottom}, grid);

//     // Labels
//     const std::vector<std::string> labels = {
//         "Frame", "Mask", "Ridge", "3D Points (XZ)"
//     };
//     const std::vector<cv::Point> positions = {
//         {10,      30},
//         {w + 10,  30},
//         {10,      h + 30},
//         {w + 10,  h + 30}
//     };
//     for (size_t i = 0; i < labels.size(); ++i) {
//         cv::putText(grid, labels[i], positions[i],
//             cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 0}, 2, cv::LINE_AA);
//     }

//     return grid;
// }

std::optional<Position> Perception::get_latest_bullsey_point()
{
    // TODO: implement bullseye detection
    return std::nullopt;
}

std::optional<Position> Perception::get_latest_end_goal_point()
{
    // TODO: implement end-goal detection
    return std::nullopt;
}