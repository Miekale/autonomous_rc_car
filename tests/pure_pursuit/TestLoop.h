#ifndef TEST_LOOP_H
#define TEST_LOOP_H

#include "PurePursuit.hpp"
#include "PurePursuitControls.hpp"
#include <iostream>
#include <cassert>
#include <fstream> // Required for ofstream
#include <iomanip> // For clean decimal formatting

const double M_PI = 3.141592;

void test_circle_tracking() {
    const double R = 20.0;
    const int N = 200;
    std::vector<Position> path;

    for (int i = 0; i < N; i++) {
        double t = 2.0 * M_PI * i / N;
        path.push_back({ R * cos(t), R * sin(t), 0 });
    }

    Position robot{R + 2.0, 0, M_PI/2};
    PurePursuit pp(10.0, 2.0);
    PurePursuitControls ctrl(1.0, 0.5, 10.0, 5.0);

    double dt = 0.1;
    double maxRadialError = 0;

    // --- File Setup ---
    std::ofstream logFile("data/circle_tracking_data.csv");
    logFile << "x,y,theta,lookahead_x,lookahead_y,v,w\n"; // CSV Header

    for (int i = 0; i < 400; i++) {
        Position lookahead = pp.findLookaheadPoint(robot, path);
        auto [v, w] = ctrl.getControl(robot, lookahead, path);

        // Save current state BEFORE moving (or after, just be consistent)
        logFile << robot.x << "," << robot.y << "," << robot.theta << ","
                << lookahead.x << "," << lookahead.y << ","
                << v << "," << w << "\n";

        // simulate motion
        robot.x += v * cos(robot.theta) * dt;
        robot.y += v * sin(robot.theta) * dt;
        robot.theta += w * dt;

        double r = std::sqrt(robot.x * robot.x + robot.y * robot.y);
        maxRadialError = std::max(maxRadialError, std::abs(r - R));

        assert(v > 0);
        assert(std::abs(w) < 5);
    }

    logFile.close();
    assert(maxRadialError < 5.0);
    std::cout << "test_circle_tracking passed! Data saved to circle_tracking_data.csv\n";
}


void simulate_figure8() {
    const double R = 20.0;
    const int N = 600;
    std::vector<Position> path;

    // --- Figure-8 path ---
    for (int i = 0; i < N; i++) {
        double t = 2.0 * M_PI * i / N;
        double x = R * sin(t);
        double y = R * sin(t) * cos(t);
        path.push_back({x, y, 0});
    }

    Position robot{R + 2.0, 0, M_PI/2};

    PurePursuit pp(7.0, 1.0);
    PurePursuitControls ctrl(0.8, 1.2, 7.0, 5.0);

    std::ofstream logFile("figure8_visualization.csv");
    logFile << "robot_x,robot_y,theta,"
               "lookahead_x,lookahead_y,"
               "path_x,path_y,v,w\n";

    double dt = 0.1;

    for (int i = 0; i < 800; i++) {
        Position lookahead = pp.findLookaheadPoint(robot, path);
        auto [v, w] = ctrl.getControl(robot, lookahead, path);

        // log robot state
        logFile << robot.x << "," << robot.y << "," << robot.theta << ","
                << lookahead.x << "," << lookahead.y << ","
                << path[i % path.size()].x << ","
                << path[i % path.size()].y << ","
                << v << "," << w << "\n";

        // simulate robot motion
        robot.x += v * cos(robot.theta) * dt;
        robot.y += v * sin(robot.theta) * dt;
        robot.theta += w * dt;
    }

    logFile.close();
    std::cout << "Figure-8 data saved to data/figure8_visualization.csv\n";
}

void test_loop() {
    test_circle_tracking();
    simulate_figure8();
}


#endif