#include "Perception.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    std::string source;
    if (argc >= 2) {
        source = argv[1];
    }

    cv::VideoCapture cap;
    if (!source.empty()) {
        cap.open(source);
    } else {
        cap.open(0);
    }

    if (!cap.isOpened()) {
        std::cerr << "Failed to open video source\n";
        return 1;
    }

    Perception perception(0);

    cv::Mat frame;
    if (!cap.read(frame) || frame.empty()) {
        std::cerr << "Failed to read first frame\n";
        return 1;
    }

    while (cap.isOpened()) {
        const int height = 600;
        const int width = static_cast<int>(static_cast<double>(frame.cols) * (static_cast<double>(height) / static_cast<double>(frame.rows)));

        cv::imshow("Frame", frame);

        if (!cap.read(frame) || frame.empty()) {
            break;
        }
        cv::resize(frame, frame, cv::Size(width, height));

        perception.get_latest_bgr_frame();
        const std::vector<Position> points = perception.get_latest_line_follow_points();

        const int topdown_h = 600;
        const int topdown_w = 600;
        cv::Mat topdown(topdown_h, topdown_w, CV_8UC3, cv::Scalar(0, 0, 0));

        const double xmin = -100.0;
        const double xmax = 100.0;
        const double ymin = -100.0;
        const double ymax = 100.0;

        const double xr = std::max(xmax - xmin, 1e-6);
        const double yr = std::max(ymax - ymin, 1e-6);

        for (const auto& p : points) {
            const double u_f = (p.x - xmin) / xr * (topdown_w - 1);
            const double v_f = (p.y - ymin) / yr * (topdown_h - 1);
            const int u = static_cast<int>(std::lround(u_f));
            const int v = (topdown_h - 1) - static_cast<int>(std::lround(v_f));
            if (0 <= u && u < topdown_w && 0 <= v && v < topdown_h) {
                cv::circle(topdown, cv::Point(u, v), 1, cv::Scalar(0, 255, 0), -1);
            }
        }

        cv::putText(
            topdown,
            "Top Down: x[-100,100]  y[-100,100]",
            cv::Point(10, 20),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(255, 255, 255),
            1,
            cv::LINE_AA
        );

        cv::circle(topdown, cv::Point(20, 40), 4, cv::Scalar(0, 255, 0), -1);
        cv::putText(
            topdown,
            "Detected points",
            cv::Point(30, 44),
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            cv::Scalar(255, 255, 255),
            1,
            cv::LINE_AA
        );

        const double tick_step = 50.0;
        const int tick_len = 8;
        const double font_scale = 0.45;
        const int font_thickness = 1;
        const cv::Scalar text_color(255, 255, 255);
        const cv::Scalar tick_color(180, 180, 180);

        for (double xw = xmin; xw <= xmax + 1e-9; xw += tick_step) {
            const int u_tick = static_cast<int>(std::lround((xw - xmin) / xr * (topdown_w - 1)));
            if (0 <= u_tick && u_tick < topdown_w) {
                cv::line(topdown, cv::Point(u_tick, topdown_h - 1), cv::Point(u_tick, topdown_h - 1 - tick_len), tick_color, 1);
                cv::putText(
                    topdown,
                    std::to_string(static_cast<int>(xw)),
                    cv::Point(u_tick - 12, topdown_h - 1 - tick_len - 5),
                    cv::FONT_HERSHEY_SIMPLEX,
                    font_scale,
                    text_color,
                    font_thickness,
                    cv::LINE_AA
                );
            }
        }

        for (double yw = ymin; yw <= ymax + 1e-9; yw += tick_step) {
            int v_tick = static_cast<int>(std::lround((yw - ymin) / yr * (topdown_h - 1)));
            v_tick = (topdown_h - 1) - v_tick;
            if (0 <= v_tick && v_tick < topdown_h) {
                cv::line(topdown, cv::Point(0, v_tick), cv::Point(tick_len, v_tick), tick_color, 1);
                cv::putText(
                    topdown,
                    std::to_string(static_cast<int>(yw)),
                    cv::Point(tick_len + 3, v_tick + 4),
                    cv::FONT_HERSHEY_SIMPLEX,
                    font_scale,
                    text_color,
                    font_thickness,
                    cv::LINE_AA
                );
            }
        }

        cv::imshow("Top Down", topdown);

        const int key = cv::waitKey(1);
        if ((key & 0xFF) == 'q') {
            break;
        }
    }

    cap.release();
    return 0;
}