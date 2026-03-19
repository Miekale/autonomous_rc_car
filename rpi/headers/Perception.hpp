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

#include "Constants.hpp"  // defines Position

class Perception {
public:
    // Bundles all outputs from the detection pipeline
    struct DetectionResult {
        cv::Mat             mask;       // binary red mask (after morphology)
        cv::Mat             ridge;      // medial-axis / ridge image
        std::vector<cv::Point2f>  points_2d;  // ridge pixels
        std::vector<cv::Point3d>  points_3d;  // back-projected floor coords
    };

    Perception(std::string camera_device, bool video_file = false);
    ~Perception();

    // ── Frame injection (e.g. from a ROS callback or capture thread) ──────────
    cv::Mat get_latest_bgr_frame();

    // ── Full pipeline (thread-safe, works on any image) ───────────────────────
    DetectionResult detect_line(const cv::Mat& bgr_image,
                                int            height_filter,
                                bool           debug = false) const;

    // ── FSM / controller interface ────────────────────────────────────────────
    std::vector<Position>   get_latest_line_follow_points();
    std::optional<Position> get_latest_bullsey_point();
    std::optional<Position> get_latest_end_goal_point();
    // cv::Mat makeDebugView(
    //         const cv::Mat& frame_in,
    //         const cv::Mat& mask,
    //         const cv::Mat& ridge,
    //         const std::vector<cv::Point3f>& points_3d,
    //         int w, int h);

private:
    // ── Camera parameters ─────────────────────────────────────────────────────
    cv::Mat _camera_matrix;
    cv::Mat _dist_coeffs;
    double  _mounting_height = 0.0;

    // ── Line follow height filter ─────────────────────────────────────────────
    double HEIGHT_FILTER = 0;

    // ── HSV thresholds ────────────────────────────────────────────────────────
    cv::Scalar _lower_A, _upper_A;   // wrap-around red (174–179)
    cv::Scalar _lower_B, _upper_B;   // orange-red     (  0– 14)

    // ── Latest frame (written by set_latest_bgr_frame) ───────────────────────
    cv::VideoCapture _cap;
    mutable std::mutex _mtx;
    cv::Mat            _latest_bgr_frame;
    bool               _has_frame = false;

    // ── Pipeline steps ────────────────────────────────────────────────────────
    cv::Mat                  get_red_mask   (const cv::Mat& bgr_image) const;
    cv::Mat                  clean_mask     (const cv::Mat& mask)       const;
    cv::Mat                  extract_ridge  (const cv::Mat& mask, int height_filter) const;
    std::vector<cv::Point2f> extract_points (const cv::Mat& ridge)      const;
    std::vector<cv::Point3d> points2d_to_3d (const std::vector<cv::Point2f>& pts2d) const;
};

#endif // PERCEPTION_HPP