#include "Perception.hpp"

#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <stdexcept>
#include <iostream>
#include <chrono>
#include <cmath>

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

Perception::Perception(std::string camera_device, bool video_file, bool show_debug_plots)
{
    if (video_file) {
        std::cout << "looking for: " << camera_device << std::endl;
        _cap = cv::VideoCapture(camera_device);
    } else {
        _cap = cv::VideoCapture(std::stoi(camera_device));
        _cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
        _cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    }

    if (!_cap.isOpened()) {
        std::cerr << "Failed to open camera\n";
    }

    // Intrinsic parameter init
    // Flatten row-major into a 1D vector
    std::vector<double> flat;
    for (const auto& row : INTRINSIC_MATRIX)
        flat.insert(flat.end(), row.begin(), row.end());

    _camera_matrix = cv::Mat(3, 3, CV_64F, flat.data()).clone();

    // Distortion coefficients (1×N)
    std::vector<double> dist_flat(DISTORTION_COEFFICIENTS.begin(), DISTORTION_COEFFICIENTS.end());
    _dist_coeffs = cv::Mat(1, dist_flat.size(), CV_64F, dist_flat.data()).clone();

    // Latest BGR frame
    _latest_bgr_frame = cv::Mat();
    _show_debug_plots = show_debug_plots;
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

cv::Mat Perception::get_blue_mask(const cv::Mat& bgr_image) const
{
    cv::Mat blurred;
    cv::GaussianBlur(bgr_image, blurred, cv::Size(5, 5), 0);

    cv::Mat hsv;
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv, _lower_blue, _upper_blue, mask);
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

cv::Mat Perception::extract_ridge(const cv::Mat& mask, int height_filter, int width_filter) const
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

    
    // defualt cut off none - but can cut off side points on the side horizontally
    const int x_margin = ridge.cols * width_filter / 100;
    ridge(cv::Rect(0, 0, x_margin, ridge.rows)).setTo(0);
    ridge(cv::Rect(ridge.cols - x_margin, 0, x_margin, ridge.rows)).setTo(0);
    

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

    // // Undistort pixel coordinates
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

        if (Z <= 0 || Z > 2000) continue;

        pts3d.emplace_back(X, Y, Z);
    }
    return pts3d;
}

cv::Mat Perception::render_xz_plot(const std::vector<cv::Point3d>& pts3d,
                                   int w, int h,
                                   float x_min, float x_max,
                                   float z_min, float z_max) const
{
    cv::Mat plot(h, w, CV_8UC3, cv::Scalar(30, 30, 30));

    // Draw axis lines.
    const int cx = w / 2;
    cv::line(plot, {cx, 0}, {cx, h}, cv::Scalar(80, 80, 80), 1);
    cv::line(plot, {0, h - 1}, {w, h - 1}, cv::Scalar(80, 80, 80), 1);

    // Labels.
    cv::putText(plot, "X", {cx + 4, 14},
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1);
    cv::putText(plot, "Z", {4, h - 4},
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1);

    for (const auto& p : pts3d)
    {
        const float px = (p.x - x_min) / (x_max - x_min) * w;
        const float py = h - (p.z - z_min) / (z_max - z_min) * h;
        const int ix = static_cast<int>(px);
        const int iy = static_cast<int>(py);

        if (ix >= 0 && ix < w && iy >= 0 && iy < h) {
            cv::circle(plot, {ix, iy}, 2, cv::Scalar(0, 200, 255), -1);
        }
    }

    return plot;
}
cv::Mat Perception::make_debug_grid(const cv::Mat& frame,
                                    const cv::Mat& mask,
                                    const cv::Mat& ridge,
                                    const std::vector<cv::Point3d>& pts3d) const
{
    std::cout << "making debug grid" << std::endl;
    const int w = frame.cols;
    const int h = frame.rows;

    auto fit = [&](const cv::Mat& img) {
        cv::Mat out;
        cv::resize(img, out, {w, h}, 0, 0, cv::INTER_NEAREST);
        return out;
    };

    cv::Mat mask_bgr, ridge_bgr;
    cv::cvtColor(mask, mask_bgr, cv::COLOR_GRAY2BGR);
    cv::cvtColor(ridge, ridge_bgr, cv::COLOR_GRAY2BGR);
    cv::Mat plot_bgr = render_xz_plot(pts3d, w, h);

    cv::Mat top, bottom, grid;
    cv::hconcat(std::vector<cv::Mat>{frame, fit(mask_bgr)}, top);
    cv::hconcat(std::vector<cv::Mat>{fit(ridge_bgr), fit(plot_bgr)}, bottom);
    cv::vconcat(std::vector<cv::Mat>{top, bottom}, grid);

    const std::vector<std::string> labels = {"Frame", "Mask", "Ridge", "3D Points (XZ)"};
    const std::vector<cv::Point> positions = {
        {10, 30},
        {w + 10, 30},
        {10, h + 30},
        {w + 10, h + 30}
    };
    for (size_t i = 0; i < labels.size(); ++i) {
        cv::putText(grid, labels[i], positions[i], cv::FONT_HERSHEY_SIMPLEX,
                    0.8, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    }

    cv::resize(grid, grid, {int(1920 / 1.5), int(1080 / 1.5)}, 0, 0, cv::INTER_NEAREST);
    return grid;
}

// ─────────────────────────────────────────────────────────────────────────────
// Public pipeline – detect_line
// ─────────────────────────────────────────────────────────────────────────────
Perception::DetectionResult 
Perception::detect_line(const cv::Mat& bgr_image, int height_filter, bool debug)
{
    double t0 = now_sec();

    cv::Mat mask  = get_red_mask(bgr_image);          double t1 = now_sec();
    mask          = clean_mask(mask);                  double t2 = now_sec();
    cv::Mat ridge = extract_ridge(mask, height_filter);double t3 = now_sec();
    auto pts2d    = extract_points(ridge);             double t4 = now_sec();
    auto pts3d    = points2d_to_3d(pts2d);             double t5 = now_sec();

    // if (debug)
    // {
    //     std::cout << "=======================\n"
    //               << "Detection took   " << (t5 - t0) << " s\n"
    //               << "Red mask         " << (t1 - t0) << " s\n"
    //               << "Clean mask       " << (t2 - t1) << " s\n"
    //               << "Extract ridge    " << (t3 - t2) << " s\n"
    //               << "Extract points   " << (t4 - t3) << " s\n"
    //               << "Points to 3-D    " << (t5 - t4) << " s\n"
    //               << "=======================\n";
    // }

    if (_show_debug_plots) {
        cv::Mat grid = make_debug_grid(bgr_image, mask, ridge, pts3d);
        cv::imshow("Perception Debug", grid);

        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
            _show_debug_plots = false;
        };   // q or ESC
    }

    return { mask, ridge, pts2d, pts3d };
}

void Perception::set_debug_plots_enabled(bool enabled)
{
    _show_debug_plots = enabled;
}


cv::Mat Perception::get_latest_bgr_frame()
{
    std::lock_guard<std::mutex> lock(_mtx);
    _cap.read(_latest_bgr_frame);
    _has_frame = true;
    
    return _latest_bgr_frame;
}

std::vector<Position> Perception::get_latest_line_follow_points()
{
    cv::Mat frame;
    {
        std::lock_guard<std::mutex> lock(_mtx);
        if (!_has_frame) {
            return {};
        }
        
        frame = _latest_bgr_frame;
    }

    std::cout << "get_latest_line_follow_points: calling detect line" << std::endl;
    auto result = detect_line(frame, HEIGHT_FILTER, /*debug=*/false);

    std::vector<Position> positions;
    positions.reserve(result.points_3d.size());
    for (const auto& p : result.points_3d)
        positions.push_back({p.x, p.z, 0});

    return positions;
}

std::vector<Position> Perception::get_latest_line_follow_points_2d()
{
    cv::Mat frame;
    {
        std::lock_guard<std::mutex> lock(_mtx);
        if (!_has_frame) {
            return {};
        }
        
        frame = _latest_bgr_frame;
    }

    std::cout << "get_latest_line_follow_points: calling detect line" << std::endl;
    auto result = detect_line(frame, HEIGHT_FILTER, /*debug=*/false);

    std::vector<Position> positions;
    positions.reserve(result.points_2d.size());
    for (const auto& p : result.points_2d)
        positions.push_back({1080 - p.y, p.x - 1920/2, 0});

    return positions;
}

std::optional<Position> Perception::get_latest_bullsey_point()
{
    double t0 = now_sec();
    cv::Mat blue_mask = get_blue_mask(_latest_bgr_frame);

    double t1 = now_sec();
    blue_mask = clean_mask(blue_mask);

    double t2 = now_sec();
    cv::Mat blue_ridge = extract_ridge(blue_mask, 0, 10);

    double t3 = now_sec();
    auto blue_pts2d = extract_points(blue_ridge);

    double t4 = now_sec();
    std::optional<cv::Point2f> blue_center = get_center_point(blue_pts2d);

    double t5 = now_sec();

    if (_show_debug_plots)
    {
        std::cout << "3D blue points: " << blue_pts2d.size() << std::endl;

        std::cout << "=======================\n"
                  << "End goal detection took   " << (t5 - t0) << " s\n"
                  << "Mask             " << (t1 - t0) << " s\n"
                  << "Clean            " << (t2 - t1) << " s\n"
                  << "Ridge            " << (t3 - t2) << " s\n"
                  << "Extract points   " << (t4 - t3) << " s\n"
                  << "Center detection " << (t5 - t4) << " s\n"
                << "=======================\n";
    }

    if (blue_center.has_value()) {
        Position center_camera_frame = Position{ _latest_bgr_frame.rows - blue_center.value().y, blue_center.value().x - _latest_bgr_frame.cols / 2, 0};
        if (center_camera_frame.x < BULLSEYE_DISTANCE_STOP_LF) {
            return Position{0, 0, 0};
        }
        return center_camera_frame;
    }

    return std::nullopt;
}

std::optional<Position> Perception::get_latest_end_goal_point()
{
    // double t0 = now_sec();
    // cv::Mat blue_mask = get_blue_mask(_latest_bgr_frame);

    // double t1 = now_sec();
    // blue_mask = clean_mask(blue_mask);

    // double t2 = now_sec();
    // cv::Mat blue_ridge = extract_ridge(blue_mask, 0, 10);

    // double t3 = now_sec();
    // auto blue_pts2d = extract_points(blue_ridge);

    // double t4 = now_sec();
    // std::optional<cv::Point2f> blue_center = get_center_point(blue_pts2d);

    // double t5 = now_sec();

    // if (_show_debug_plots)
    // {
    //     std::cout << "3D blue points: " << blue_pts2d.size() << std::endl;

    //     std::cout << "=======================\n"
    //               << "End goal detection took   " << (t5 - t0) << " s\n"
    //               << "Mask             " << (t1 - t0) << " s\n"
    //               << "Clean            " << (t2 - t1) << " s\n"
    //               << "Ridge            " << (t3 - t2) << " s\n"
    //               << "Extract points   " << (t4 - t3) << " s\n"
    //               << "Center detection " << (t5 - t4) << " s\n"
    //             << "=======================\n";
    // }

    // if (blue_center.has_value()) {
    //     Position center_camera_frame = Position{ _latest_bgr_frame.rows - blue_center.value().y, blue_center.value().x - _latest_bgr_frame.cols / 2, 0};
    //     if (center_camera_frame.x < 100) {
    //         return Position{0, 0, 0};
    //     }
    //     return center_camera_frame;
    // }

    return std::nullopt;
}

std::optional<cv::Point2f> Perception::get_center_point(std::vector<cv::Point2f>& pts2d/*, vector<cv::Point3d>& blue_pts3d*/) const
{
    // 2D centroid from pts2d
    if (pts2d.empty()) {
        return std::nullopt;
    }

    cv::Point2f center_2d(0, 0);
    for (auto& pt : pts2d) {
        center_2d += cv::Point2f(pt.x, pt.y);
    }

    center_2d /= static_cast<float>(pts2d.size());
    
    return center_2d;

    // OR 3D centroid from blue_pts3d
    // Eigen::Vector3f blue_center_3d = Eigen::Vector3f::Zero();
    // for (auto& pt : blue_pts3d) {
    //     blue_center_3d += pt;
    // }
    // blue_center_3d /= blue_pts3d.size();
}
