#ifndef PERCEPTION_HPP
#define PERCEPTION_HPP

#include <mutex>
#include <vector>
#include <optional>
#include <atomic>
#include <thread>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "Constants.hpp"

class Perception {
private:
    mutable std::mutex _mtx;

    cv::Mat _latest_bgr_frame;
    bool _has_frame = false;

    cv::VideoCapture _cap;
    std::thread _capture_thread;
    std::atomic<bool> _running{false};

    cv::Mat _camera_matrix;
    cv::Mat _dist_coeffs;
    double _mounting_height = 0.0;

    std::vector<Position> line_detection_ridge(const cv::Mat& bgr_image);
    std::vector<cv::Point3d> points2d_to_3d(const std::vector<cv::Point2f>& points_2d) const;
    void capture_loop();
public:
    Perception(std::string video_port);
    ~Perception();

    void set_latest_bgr_frame(const cv::Mat& bgr_frame);

    // Interface to FSM & controllers
    // TODO: miekale you can change these names but make sure to re-name the usages
    // inside FSM as well
    std::vector<Position> get_latest_line_follow_points();
    std::optional<Position> get_latest_bullsey_point();
    std::optional<Position> get_latest_end_goal_point();
};

#endif
