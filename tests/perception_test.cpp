#include "Perception.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

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

    Perception perception(device, true, true);

    bool debug = true;

    while (true)
    {
        auto frame = perception.get_latest_bgr_frame();

        auto result = perception.detect_line(frame, height_filter, debug);

        // Print point count each frame
        std::cout << "\r3D points: " << result.points_3d.size()
                  << "   " << std::flush;

        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) break;   // q or ESC
        if (key == 'd') debug = !debug;        // toggle timing output
    }

    cv::destroyAllWindows();
    std::cout << "\nDone.\n";
    return 0;
}