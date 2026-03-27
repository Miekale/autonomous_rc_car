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

    struct Segment {
        cv::Point2f p1, p2;
        cv::Point2f dir;
        float len;
        int min_index;
        int max_index;
    };

    struct EndPoint {
        cv::Point2f junction;
        Segment line_straight;
        Segment line_left;
        Segment line_right;
    };

    // Bundles all outputs from the detection pipeline
    struct DetectionResult {
        DetectionResult(const cv::Mat &mask, const cv::Mat &ridge, const std::vector<cv::Point2f> &points_2d, const std::vector<cv::Point3d> &points_3d) 
            : mask(mask), ridge(ridge), points_2d(points_2d), points_3d(points_3d) {};
        DetectionResult() {};
        cv::Mat             mask;       // binary red mask (after morphology)
        cv::Mat             ridge;      // medial-axis / ridge image
        std::vector<cv::Point2f>  points_2d;  // ridge pixels
        std::vector<cv::Point3d>  points_3d;  // back-projected floor coords
    };

    Perception(std::string camera_device,
               bool        video_file = false,
               bool        debug = false,
               bool        file_saving = false);
    ~Perception();

    // ── Frame injection (e.g. from a ROS callback or capture thread) ──────────
    cv::Mat get_latest_bgr_frame();

    // ── Full pipeline (thread-safe, works on any image) ───────────────────────
    std::vector<cv::Vec3f> detect_bullseye();
    DetectionResult detect_line(const cv::Mat& bgr_image,
                                int            height_filter);
    void set_debug(bool enabled);

    // ── FSM / controller interface ────────────────────────────────────────────
    std::vector<Position>   get_latest_line_follow_points();
    std::vector<Position>   get_latest_line_follow_points_2d();
    std::optional<Position> get_latest_bullsey_point();
    std::optional<Position> get_latest_end_goal_point();

    void make_debug_grid_with_pursuit(
        const std::optional<Position>& target_point = std::nullopt,
        const std::optional<Position>& bullseye = std::nullopt,
        const std::optional<Position>& end_goal = std::nullopt
        );

    // ── Utility functions ─────────────────────────────────────────────────────
    cv::Point2d robot_to_image(int x, int y);
    cv::Point2d image_to_robot(int x, int y);

private:
    // ── Camera parameters ─────────────────────────────────────────────────────
    cv::Mat _camera_matrix;
    cv::Mat _dist_coeffs;
    double  _mounting_height = MOUNTING_HEIGHT;

    // ── Line follow height filter ─────────────────────────────────────────────
    double HEIGHT_FILTER = 0;

    int MIN_POINTS = 3; // min points for a cluster to be considered a valid line

    int FRAME_WIDTH = 1920;
    int FRAME_HEIGHT = 1080;

    // ── HSV thresholds ────────────────────────────────────────────────────────
    // Part A: "red" wrap-around (174–179)
    const cv::Scalar _lower_A = cv::Scalar(170, 50, 200);
    const cv::Scalar _upper_A = cv::Scalar(179, 255, 255);

    // Part B: "orange" red  (0–14)
    const cv::Scalar _lower_B = cv::Scalar(0, 100, 200);
    const cv::Scalar _upper_B = cv::Scalar(10, 255, 255);

    const cv::Scalar _lower_blue = cv::Scalar(110, 110, 40);
    const cv::Scalar _upper_blue = cv::Scalar(135, 255, 255);


    // ── Latest frame (written by set_latest_bgr_frame) ───────────────────────
    cv::VideoCapture _cap;
    cv::VideoWriter _video_writer;
    mutable std::mutex _mtx;
    cv::Mat            _latest_bgr_frame;
    DetectionResult    _latest_detection;
    bool               _has_frame = false;
    bool               _debug = false;
    bool               _file_saving;

    // ── Pipeline steps ────────────────────────────────────────────────────────
    cv::Mat                  get_red_mask   (const cv::Mat& bgr_image) const;
    cv::Mat                  get_blue_mask  (const cv::Mat& bgr_image) const;
    cv::Mat                  clean_mask     (const cv::Mat& mask)       const;
    cv::Mat                  extract_ridge  (const cv::Mat& mask, int height_filter, int width_filter = 0) const;
    std::vector<cv::Point2f> extract_points (const cv::Mat& ridge)      const;
    std::vector<cv::Point3d> points2d_to_3d (const std::vector<cv::Point2f>& pts2d) const;
    cv::Mat                  render_xz_plot (const std::vector<cv::Point3d>& pts3d,
                                             int w, int h,
                                             float x_min = -2000.f, float x_max = 2000.f,
                                             float z_min = 0.f, float z_max = 8000.f) const;
    
    std::optional<cv::Point2f> get_center_point(std::vector<cv::Point2f>& pts2d/*, vector<cv::Point3d>& blue_pts3d*/) const;

    // -- END POINT YALLc
    std::vector<float> getRidgeAngles(const cv::Mat& ridge, const std::vector<cv::Point2f>& points_2d) const;
    std::vector<std::vector<cv::Point2f>> clusterRidgePoints(const std::vector<cv::Point2f>& points_2d, const std::vector<float>& angles) const;
    Segment fitSegmentToCluster(const std::vector<cv::Point2f>& cluster) const;
    cv::Point2f findIntersection(const Segment& s1, const Segment& s2) const;
    std::vector<Segment> getBestSegments(const std::vector<std::vector<cv::Point2f>>& clusters) const;
    bool isTJunction(const Segment& bar, const Segment& stem, const cv::Point2f& junction) const;
    EndPoint buildEndPoint(const Segment& bar, const Segment& stem, const cv::Point2f& junction) const;
    std::optional<EndPoint> getEndpoint(const cv::Mat& ridge, const std::vector<cv::Point2f>& points_2d) const;
    bool shouldMerge(const Segment& s1, const Segment& s2) const;

};

#endif
