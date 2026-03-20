#include "Perception.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

// ─────────────────────────────────────────────────────────────────────────────
// Render a top-down XZ scatter plot of 3D points into a BGR mat
// ─────────────────────────────────────────────────────────────────────────────

static cv::Mat render_xy_plot(
    const std::vector<cv::Point3d>& pts3d,
    int w, int h,
    float x_min = -1000.f, float x_max = 1000.f,
    float z_min =     0.f, float z_max = 2000.f)
{
    cv::Mat plot(h, w, CV_8UC3, cv::Scalar(30, 30, 30));

    // Draw axis lines
    int cx = w / 2;
    cv::line(plot, {cx, 0}, {cx, h}, cv::Scalar(80, 80, 80), 1);
    cv::line(plot, {0, h - 1}, {w, h - 1}, cv::Scalar(80, 80, 80), 1);

    // Labels
    cv::putText(plot, "X", {cx + 4, 14},
        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1);
    cv::putText(plot, "Z", {4, h - 4},
        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(150, 150, 150), 1);

    for (const auto& p : pts3d)
    {
        // Map world coords → pixel coords
        float px = (p.x - x_min) / (x_max - x_min) * w;
        float py = h - (p.z - z_min) / (z_max - z_min) * h; // flip Y

        int ix = static_cast<int>(px);
        int iy = static_cast<int>(py);

        if (ix >= 0 && ix < w && iy >= 0 && iy < h)
            cv::circle(plot, {ix, iy}, 2, cv::Scalar(0, 200, 255), -1);
    }

    return plot;
}

// ─────────────────────────────────────────────────────────────────────────────
// Build the 2×2 debug grid:
//   [ frame    | mask  ]
//   [ ridge    | plot  ]
// ─────────────────────────────────────────────────────────────────────────────

static cv::Mat make_debug_grid(
    const cv::Mat& frame,
    const cv::Mat& mask,
    const cv::Mat& ridge,
    const std::vector<cv::Point3d>& pts3d,
    const std::optional<std::vector<cv::Vec3f>>& circles)
{
    const int w = frame.cols;
    const int h = frame.rows;

    auto fit = [&](const cv::Mat& img) {
        cv::Mat out;
        cv::resize(img, out, {w, h}, 0, 0, cv::INTER_NEAREST);
        return out;
    };

    if (circles.has_value()) {
        for (auto circle : circles.value()) {
            cv::circle(frame, {static_cast<int>(circle[0]), static_cast<int>(circle[1])}, static_cast<int>(circle[2]), cv::Scalar(0, 255, 0), 2);
        }
    }

    // Convert single-channel images to BGR for stacking
    cv::Mat mask_bgr, ridge_bgr;
    cv::cvtColor(mask,  mask_bgr,  cv::COLOR_GRAY2BGR);
    cv::cvtColor(ridge, ridge_bgr, cv::COLOR_GRAY2BGR);

    cv::Mat plot_bgr = render_xy_plot(pts3d, w, h);

    // Stack panels
    cv::Mat top, bottom, grid;
    cv::hconcat(std::vector<cv::Mat>{frame,          fit(mask_bgr)},  top);
    cv::hconcat(std::vector<cv::Mat>{fit(ridge_bgr), fit(plot_bgr)},  bottom);
    cv::vconcat(std::vector<cv::Mat>{top, bottom}, grid);

    // Labels
    const std::vector<std::string> labels   = {"Frame", "Mask", "Ridge", "3D Points (XZ)"};
    const std::vector<cv::Point>   positions = {
        {10,      30},
        {w + 10,  30},
        {10,      h + 30},
        {w + 10,  h + 30}
    };
    for (size_t i = 0; i < labels.size(); ++i)
        cv::putText(grid, labels[i], positions[i],
            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    
    cv::resize(grid, grid, {int(1920/1.5), int(1080/1.5)}, 0, 0, cv::INTER_NEAREST);
    return grid;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main
// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    // Optionally pass a device index or video file path as argv[1]
    std::string device = {(argc > 1) ? argv[1] : "0"};
    int height_filter  = (argc > 2) ? std::stoi(argv[2]) : 0;

    std::cout << "Opening: " << device << "\n";
    std::cout << "Height filter: " << height_filter << "\n";
    std::cout << "Press 'q' or ESC to quit, 'd' to toggle timing debug\n";

    Perception perception(device, true);

    bool debug = true;

    while (true)
    {
        auto frame = perception.get_latest_bgr_frame();

        auto result = perception.detect_line(frame, height_filter, debug);
        auto circles = perception.detect_bullseye();
        std::cout << "Circles: " << circles.size() << std::endl;

        // Print point count each frame
        std::cout << "\r3D points: " << result.points_3d.size()
                  << "   " << std::flush;

        cv::Mat grid = make_debug_grid(frame, result.mask, result.ridge, result.points_3d, circles);

        cv::imshow("Perception Debug", grid);

        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) break;   // q or ESC
        if (key == 'd') debug = !debug;        // toggle timing output
    }

    cv::destroyAllWindows();
    std::cout << "\nDone.\n";
    return 0;
}