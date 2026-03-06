#include "Perception.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <chrono>
#include <thread>

Perception::Perception(std::string video_port) {
    _camera_matrix = (cv::Mat_<double>(3, 3)
        << CAMERA_INTRINSIC_MATRIX[0][0], CAMERA_INTRINSIC_MATRIX[0][1], CAMERA_INTRINSIC_MATRIX[0][2],
           CAMERA_INTRINSIC_MATRIX[1][0], CAMERA_INTRINSIC_MATRIX[1][1], CAMERA_INTRINSIC_MATRIX[1][2],
           CAMERA_INTRINSIC_MATRIX[2][0], CAMERA_INTRINSIC_MATRIX[2][1], CAMERA_INTRINSIC_MATRIX[2][2]);

    _dist_coeffs = (cv::Mat_<double>(1, 5)
        << CAMERA_DISTORTION_COEFFICIENTS[0], CAMERA_DISTORTION_COEFFICIENTS[1], CAMERA_DISTORTION_COEFFICIENTS[2],
           CAMERA_DISTORTION_COEFFICIENTS[3], CAMERA_DISTORTION_COEFFICIENTS[4]);

    _mounting_height = MOUNTING_HEIGHT;

    if (_cap.open(video_port)) {
        _running.store(true);
        _capture_thread = std::thread(&Perception::capture_loop, this);
    }
}

Perception::~Perception() {
    _running.store(false);
    if (_capture_thread.joinable()) {
        _capture_thread.join();
    }
    if (_cap.isOpened()) {
        _cap.release();
    }
}

void Perception::capture_loop() {
    while (_running.load()) {
        cv::Mat frame;
        if (!_cap.isOpened()) {
            set_latest_bgr_frame(cv::Mat());
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        if (!_cap.read(frame) || frame.empty()) {
            set_latest_bgr_frame(cv::Mat());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        set_latest_bgr_frame(frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void Perception::set_latest_bgr_frame(const cv::Mat& bgr_frame) {
    std::lock_guard<std::mutex> lk(_mtx);
    if (bgr_frame.empty()) {
        _latest_bgr_frame.release();
        _has_frame = false;
        return;
    }
    bgr_frame.copyTo(_latest_bgr_frame);
    _has_frame = true;
}

std::vector<cv::Point3d> Perception::points2d_to_3d(const std::vector<cv::Point2f>& points_2d) const {
    std::vector<cv::Point3d> out;
    if (points_2d.empty()) {
        return out;
    }

    cv::Mat src(static_cast<int>(points_2d.size()), 1, CV_32FC2);
    for (int i = 0; i < src.rows; ++i) {
        src.at<cv::Vec2f>(i, 0) = cv::Vec2f(points_2d[static_cast<size_t>(i)].x, points_2d[static_cast<size_t>(i)].y);
    }

    cv::Mat undist;
    cv::undistortPoints(src, undist, _camera_matrix, _dist_coeffs, cv::noArray(), _camera_matrix);

    const double fx = _camera_matrix.at<double>(0, 0);
    const double fy = _camera_matrix.at<double>(1, 1);
    const double cx = _camera_matrix.at<double>(0, 2);
    const double cy = _camera_matrix.at<double>(1, 2);
    const double depth = _mounting_height;

    out.reserve(points_2d.size());
    for (int i = 0; i < undist.rows; ++i) {
        const cv::Vec2f uv = undist.at<cv::Vec2f>(i, 0);
        const double u = static_cast<double>(uv[0]);
        const double v = static_cast<double>(uv[1]);
        const double X = depth * (u - cx) / fx;
        const double Y = depth * (v - cy) / fy;
        const double Z = depth;
        out.emplace_back(X, Y, Z);
    }
    return out;
}

std::vector<Position> Perception::line_detection_ridge(const cv::Mat& bgr_image) {
    std::vector<Position> out;
    if (bgr_image.empty()) {
        return out;
    }

    cv::Mat hsv;
    cv::cvtColor(bgr_image, hsv, cv::COLOR_BGR2HSV);
    cv::GaussianBlur(hsv, hsv, cv::Size(5, 5), 0);

    const cv::Scalar red_lower(RED_LOWER[0], RED_LOWER[1], RED_LOWER[2]);
    const cv::Scalar red_upper(RED_UPPER[0], RED_UPPER[1], RED_UPPER[2]);
    cv::Mat mask;
    cv::inRange(hsv, red_lower, red_upper, mask);

    cv::Mat dist;
    cv::distanceTransform(mask, dist, cv::DIST_L2, 5);

    cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
    cv::Mat dilated;
    cv::dilate(dist, dilated, kernel);

    cv::Mat eq;
    cv::compare(dist, dilated, eq, cv::CMP_EQ);

    cv::Mat gt0;
    cv::compare(dist, 0.0, gt0, cv::CMP_GT);

    cv::Mat ridge;
    cv::bitwise_and(eq, gt0, ridge);

    std::vector<cv::Point> nz;
    cv::findNonZero(ridge, nz);
    if (nz.empty()) {
        return out;
    }

    std::vector<cv::Point2f> pts2d;
    pts2d.reserve(nz.size());
    for (const auto& p : nz) {
        pts2d.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));
    }

    const std::vector<cv::Point3d> pts3d = points2d_to_3d(pts2d);
    out.reserve(pts3d.size());
    for (const auto& p : pts3d) {
        out.push_back(Position{p.x, p.y, 0.0});
    }

    return out;
}

std::vector<Position> Perception::get_latest_line_follow_points() {
    cv::Mat frame;
    {
        std::lock_guard<std::mutex> lk(_mtx);
        if (!_has_frame || _latest_bgr_frame.empty()) {
            return {};
        }
        _latest_bgr_frame.copyTo(frame);
    }
    return line_detection_ridge(frame);
}

std::optional<Position> Perception::get_latest_bullsey_point() {
    return {};
}

std::optional<Position> Perception::get_latest_end_goal_point() {
    return {};
}