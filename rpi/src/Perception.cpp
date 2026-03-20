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
Perception::detect_line(const cv::Mat& bgr_image, int height_filter, bool debug) 
{
    double t0 = now_sec();

    cv::Mat mask  = get_red_mask(bgr_image);          double t1 = now_sec();
    mask          = clean_mask(mask);                  double t2 = now_sec();
    cv::Mat ridge = extract_ridge(mask, height_filter);double t3 = now_sec();
    auto pts2d    = extract_points(ridge);             double t4 = now_sec();
    auto pts3d    = points2d_to_3d(pts2d);             double t5 = now_sec();

    auto endpoint = getEndpoint(ridge, pts2d);
    if (endpoint) {
        std::cout << "Found endpoint at: " << endpoint->junction << "\n";
    } else {
        std::cout << "No endpoint found\n";
    }

    if (debug)
    {
        std::cout << "=======================\n"
                  << "Detection took   " << (t5 - t0) << " s\n"
                  << "Red mask         " << (t1 - t0) << " s\n"
                  << "Clean mask       " << (t2 - t1) << " s\n"
                  << "Extract ridge    " << (t3 - t2) << " s\n"
                  << "Extract points   " << (t4 - t3) << " s\n"
                  << "Points to 3-D    " << (t5 - t4) << " s\n"
                //   << "num of clusters       " << clusters.size() << " s\n"
                  << "=======================\n";
    }

    _latest_detection = { mask, ridge, pts2d, pts3d };
    return _latest_detection;
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
std::optional<Position> Perception::get_latest_bullsey_point()
{
    // std::vector<cv::Vec3f> circles;
    // cv::HoughCircles(_latest_bgr_frame,   // <-- see point 2 below
    //                  circles, 
    //                  cv::HOUGH_GRADIENT, 
    //                  1,
    //                  _latest_bgr_frame.rows / 4,
    //                  100, 30, 
    //                  20, 200);
    
    // if (!circles.empty()) {
    //     return Position{circles[0][0], circles[0][1], circles[0][2]};  // adjust field names to match your struct
     
    // }

    return std::nullopt;
}

std::optional<Position> Perception::get_latest_end_goal_point()
{
    // TODO: implement end-goal detection
    return std::nullopt;
}

std::vector<float> Perception::getRidgeAngles(const cv::Mat& ridge, const std::vector<cv::Point2f>& points_2d) const
/*
Sobel should get the gradient of the ridge image, which is perpindicular to the edge
we then rotate to get the angle of the edge itself, as those two properties are 90 degrees apart
*/
{
    cv::Mat dx, dy;
    cv::Sobel(ridge, dx, CV_32F, 1, 0, 3);
    cv::Sobel(ridge, dy, CV_32F, 0, 1, 3);

    std::vector<float> angles;
    angles.reserve(points_2d.size());

    for(auto&p : points_2d) {
        int x = (int)p.x;
        int y = (int)p.y;

        float gx = dx.at<float>(y, x);
        float gy = dy.at<float>(y, x);
        float angle = std::atan2(gx, -gy);
        if (angle < 0) angle += CV_PI; // atan2 returns [-pi, pi], we want [0, pi]

        angles.push_back(angle);
    }
    return angles;
}

std::vector<std::vector<cv::Point2f>> Perception::clusterRidgePoints(const std::vector<cv::Point2f>& points_2d, const std::vector<float>& angles) const
/*
We bucket the points by their angle, the two largest buckets would be the primary lines
*/ 
{
    int bins = 4; // arb
    std::vector<std::vector<cv::Point2f>> clusters(bins);

    for (size_t i = 0; i < points_2d.size(); ++i) {
        float angle = angles[i];
        int bin = (int)(angle / CV_PI * bins) % bins;
        clusters[bin].push_back(points_2d[i]);
    }
    return clusters;
}


Perception::Segment Perception::fitSegmentToCluster(const std::vector<cv::Point2f>& cluster) const
/*
Called after clusterRidgePoints, operate on each bin and fit a line to the points in that bucket
https://docs.opencv.org/3.4/js_contour_features_fitLine.html foir fitline

only run if there are points inside lol
*/
{
    cv::Vec4f line;
    // get a unit direction and point on the line representing the clusters trend
    cv::fitLine(cluster, line, cv::DIST_L2, 0, 0.01, 0.01);

    cv::Point2f direction = {line[0], line[1]}; // vx, vy
    cv::Point2f point_on_line = {line[2], line[3]}; // x0, y0

    float tmin = std::numeric_limits<float>::infinity();
    float tmax = -std::numeric_limits<float>::infinity();

    int min_index = -1;
    int max_index = -1;

    for (int i = 0; i < (int)cluster.size(); i++) {
        float t = direction.dot(cluster[i] - point_on_line);
        if (t < tmin) {
            tmin = t;
            min_index = i;
        }
        if (t > tmax) {
            tmax = t;
            max_index = i;
        }
    }

    Segment clusterSegment;
    clusterSegment.p1 = point_on_line + tmin * direction; // negative ot it
    clusterSegment.p2 = point_on_line + tmax * direction;
    clusterSegment.dir = direction;
    clusterSegment.len    = tmax - tmin;
    clusterSegment.min_index = min_index;
    clusterSegment.max_index = max_index;

    return clusterSegment;
}

/*
need a function that loops through the segments and finds which ones are parallel
*/

std::vector<Perception::Segment> Perception::getBestSegments(const std::vector<std::vector<cv::Point2f>>& clusters) const
/*
find top 2 segments
*/
{
    std::vector<Segment> significant_segments;
    auto sorted = clusters;
    std::sort(sorted.begin(), sorted.end(), 
        [](const auto& a, const auto& b){ return a.size() > b.size(); });

    for (int i = 0; i < 2 && i < sorted.size(); i++) {
        if (sorted[i].size() > MIN_POINTS)  // filter noise
            significant_segments.push_back(fitSegmentToCluster(sorted[i]));
    }

    return significant_segments;
}

cv::Point2f Perception::findIntersection(const Segment& s1, const Segment& s2) const {
    // Solve: s1.p1 + t*s1.dir = s2.p1 + u*s2.dir
    cv::Point2f dp = s2.p1 - s1.p1;
    float cross = s1.dir.x * s2.dir.y - s1.dir.y * s2.dir.x;
    float t = (dp.x * s2.dir.y - dp.y * s2.dir.x) / cross;
    return s1.p1 + t * s1.dir;
}

// bool Perception::isTJunction(const Segment& bar, const Segment& stem, const cv::Point2f& junction) const {
//     // Project junction onto each segment, check normalized position [0,1]
//     float t_bar  = bar.dir.dot(junction - bar.p1) / bar.len;
//     float t_stem = stem.dir.dot(junction - stem.p1) / stem.len;

//     bool junction_on_bar_middle = (t_bar  > 0.2f && t_bar  < 0.8f);
//     bool junction_at_stem_end   = (t_stem < 0.2f || t_stem > 0.8f);

//     return junction_on_bar_middle && junction_at_stem_end;
// }

bool Perception::isTJunction(const Segment& bar, const Segment& stem, const cv::Point2f& junction) const {
    float t_bar  = bar.dir.dot(junction - bar.p1) / bar.len;
    float t_stem = stem.dir.dot(junction - stem.p1) / stem.len;

    std::cout << "isTJunction t_bar=" << t_bar << "  t_stem=" << t_stem << "\n";

    // Junction must land in middle third of bar (not near ends)
    bool junction_on_bar_middle = (t_bar > 0.3f && t_bar < 0.7f);

    // Junction must land near the end of stem (within 20%)
    bool junction_at_stem_end = (t_stem < 0.2f || t_stem > 0.8f);

    // Bar must be significantly longer than stem
    bool bar_longer = (bar.len > stem.len * 1.5f);

    // Bar must be roughly horizontal (dir.x dominates)
    bool bar_horizontal = (std::abs(bar.dir.x) > 0.7f);

    // Stem must be roughly vertical (dir.y dominates)  
    bool stem_vertical = (std::abs(stem.dir.y) > 0.7f);

    std::cout << "  bar_middle=" << junction_on_bar_middle
              << "  stem_end=" << junction_at_stem_end
              << "  bar_longer=" << bar_longer
              << "  bar_horiz=" << bar_horizontal
              << "  stem_vert=" << stem_vertical << "\n";

    return junction_on_bar_middle && junction_at_stem_end 
        && bar_longer && bar_horizontal && stem_vertical;
}

Perception::EndPoint Perception::buildEndPoint(const Segment& bar, const Segment& stem, const cv::Point2f& junction) const {
    EndPoint ep;
    ep.junction = junction;
    ep.line_straight = stem;

    // Left and right are the two halves of the bar
    // Split bar at junction point
    ep.line_left.p1  = junction;
    ep.line_left.p2  = bar.p1;   // one end of bar
    ep.line_right.p1 = junction;
    ep.line_right.p2 = bar.p2;   // other end of bar

    return ep;
}


bool Perception::shouldMerge(const Segment& s1, const Segment& s2) const {
    // 1. Angular similarity (within ~8 degrees)
    float dot = std::abs(s1.dir.dot(s2.dir));
    if (dot < 0.99f) return false;

    // 2. Lateral distance (Are the lines on the same "track"?)
    cv::Point2f v = s2.p1 - s1.p1;
    float dist = std::abs(v.x * s1.dir.y - v.y * s1.dir.x);
    if (dist > 15.0f) return false; // Allowed 15px of "wobble"

    // 3. Longitudinal gap (The "Bridge")
    float t2_p1 = s1.dir.dot(s2.p1 - s1.p1);
    float t2_p2 = s1.dir.dot(s2.p2 - s1.p1);
    float min_t = std::min(t2_p1, t2_p2);
    float max_t = std::max(t2_p1, t2_p2);

    // Allowing a 120px gap to bridge your Seg 0 and Seg 2 (90px apart)
    return !(min_t > s1.len + 120.0f || max_t < -120.0f);
}
std::optional<Perception::EndPoint> Perception::getEndpoint(const cv::Mat& ridge, const std::vector<cv::Point2f>& points_2d) const {
    if (points_2d.size() < 20) return std::nullopt;

    // 1. Rasterize
    cv::Mat binary = cv::Mat::zeros(ridge.size(), CV_8UC1);
    for (const auto& p : points_2d) {
        if (p.x >= 0 && p.x < binary.cols && p.y >= 0 && p.y < binary.rows)
            binary.at<uchar>((int)p.y, (int)p.x) = 255;
    }

    // 2. Hough
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(binary, lines, 1, CV_PI / 180, 10, 15, 10);
    
    std::vector<Segment> raw;
    for (const auto& l : lines) {
        Segment s; s.p1 = {(float)l[0], (float)l[1]}; s.p2 = {(float)l[2], (float)l[3]};
        cv::Point2f d = s.p2 - s.p1; s.len = cv::norm(d); s.dir = d * (1.0f / s.len);
        raw.push_back(s);
    }

    // 3. Aggressive Merge (Bridges gaps in the Bar)
    std::vector<Segment> merged;
    std::vector<bool> used(raw.size(), false);
    for (size_t i = 0; i < raw.size(); ++i) {
        if (used[i]) continue;
        Segment curr = raw[i]; used[i] = true;
        for (size_t j = i + 1; j < raw.size(); ++j) {
            if (!used[j] && shouldMerge(curr, raw[j])) {
                float t3 = curr.dir.dot(raw[j].p1 - curr.p1);
                float t4 = curr.dir.dot(raw[j].p2 - curr.p1);
                float tMin = std::min({0.0f, curr.len, t3, t4});
                float tMax = std::max({0.0f, curr.len, t3, t4});
                cv::Point2f newP1 = curr.p1 + tMin * curr.dir;
                curr.p2 = curr.p1 + tMax * curr.dir;
                curr.p1 = newP1;
                curr.len = tMax - tMin;
                used[j] = true;
            }
        }
        merged.push_back(curr);
    }

    // --- 4. T-Junction Search with Bottom-Half Constraint ---
    float y_threshold = (float)ridge.rows / 2.0f;

    for (const auto& bar : merged) {
        for (const auto& stem : merged) {
            if (&bar == &stem) continue;

            if (std::abs(bar.dir.dot(stem.dir)) > 0.45f) continue;

            cv::Point2f tips[2] = {stem.p1, stem.p2};
            for (int k = 0; k < 2; ++k) {
                float t = bar.dir.dot(tips[k] - bar.p1);
                cv::Point2f proj = bar.p1 + t * bar.dir;

                // NEW: Constraint Check
                // Only accept if the junction point is in the bottom 50% of the image
                if (proj.y < y_threshold) continue; 

                float dist = cv::norm(tips[k] - proj);
                float normT = t / bar.len;

                if (dist < 45.0f && (normT > -0.1f && normT < 1.1f)) {
                    // Success!
                    return buildEndPoint(bar, stem, proj);
                }
            }
        }
    }

    return std::nullopt;
}