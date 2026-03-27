#ifndef TEST_LOOP_H
#define TEST_LOOP_H

#include "PurePursuit.hpp"
#include <iostream>
#include <cassert>
#include <fstream> // Required for ofstream
#include <iomanip> // For clean decimal formatting

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
void test_circle_tracking() {
    const double R = 20.0;
    const int N = 200;
    std::vector<Position> path;

    for (int i = 0; i < N; i++) {
        double t = 2.0 * M_PI * i / N;
        path.push_back({ R * cos(t), R * sin(t), 0 });
    }

    Position robot{R + 2.0, 0, M_PI/2};

    PurePursuit pp(
        10.0,  // lookAheadDist
        2.0,   // lookAheadTol
        1.0,   // K_curve
        0.5,   // K_velocity
        5.0    // maxLinearVel
    );

    double dt = 0.1;
    double maxRadialError = 0;

    // --- File Setup ---
    std::ofstream logFile("data/circle_tracking_data.csv");
    logFile << "x,y,theta,lookahead_x,lookahead_y,v,w\n";

    for (int i = 0; i < 400; i++) {
        Position lookahead = pp.findLookaheadPoint(robot, path);
        auto [v, w] = pp.getControl(robot, lookahead);

        logFile << robot.x << "," << robot.y << "," << robot.theta << ","
                << lookahead.x << "," << lookahead.y << ","
                << v << "," << w << "\n";

        // simulate motion
        robot.x += v * std::cos(robot.theta) * dt;
        robot.y += v * std::sin(robot.theta) * dt;
        robot.theta += w * dt;

        double r = std::sqrt(robot.x * robot.x + robot.y * robot.y);
        maxRadialError = std::max(maxRadialError, std::abs(r - R));

        assert(v > 0);
        assert(std::abs(w) < 5);
    }

    logFile.close();
    assert(maxRadialError < 5.0);
    std::cout << "[PASS] test_circle_tracking passed! Data saved to circle_tracking_data.csv\n";
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

    PurePursuit pp(
        7.0,  // lookAheadDist
        1.0,  // lookAheadTol
        0.8,  // K_curve
        1.2,  // K_velocity
        5.0   // maxLinearVel
    );

    std::ofstream logFile("data/figure8_visualization.csv");
    logFile << "robot_x,robot_y,theta,"
               "lookahead_x,lookahead_y,"
               "path_x,path_y,v,w\n";

    double dt = 0.1;

    for (int i = 0; i < 800; i++) {
        Position lookahead = pp.findLookaheadPoint(robot, path);
        auto [v, w] = pp.getControl(robot, lookahead);

        logFile << robot.x << "," << robot.y << "," << robot.theta << ","
                << lookahead.x << "," << lookahead.y << ","
                << path[i % path.size()].x << ","
                << path[i % path.size()].y << ","
                << v << "," << w << "\n";

        robot.x += v * std::cos(robot.theta) * dt;
        robot.y += v * std::sin(robot.theta) * dt;
        robot.theta += w * dt;
    }

    logFile.close();
    std::cout << "Figure-8 data saved to data/figure8_visualization.csv\n";
}

void simulate_90_degree_corner() {
    const double segment_length = 40.0;
    const int N_per_segment = 200;
    std::vector<Position> path;

    // --- 90-Degree Corner Path ---
    // Segment 1: Straight along X-axis (0,0) to (40,0)
    for (int i = 0; i < N_per_segment; i++) {
        double x = i * (segment_length / N_per_segment);
        path.push_back({x, 0.0, 0.0});
    }

    // Segment 2: Sharp turn to Y-axis (40,0) to (40,40)
    for (int i = 0; i < N_per_segment; i++) {
        double y = i * (segment_length / N_per_segment);
        path.push_back({segment_length, y, M_PI/2});
    }

    Position robot{-5.0, 0.0, 0.0};

    PurePursuit pp(
        10.0,  // lookAheadDist
        1.0,   // lookAheadTol
        0.8,   // K_curve
        1.2,   // K_velocity
        5.0    // maxLinearVel
    );

    std::ofstream logFile("corner_visualization.csv");
    logFile << "robot_x,robot_y,theta,lookahead_x,lookahead_y,v,w\n";

    double dt = 0.1;

    for (int i = 0; i < 400; i++) {
        Position lookahead = pp.findLookaheadPoint(robot, path);
        auto [v, w] = pp.getControl(robot, lookahead);

        logFile << robot.x << "," << robot.y << "," << robot.theta << ","
                << lookahead.x << "," << lookahead.y << ","
                << v << "," << w << "\n";

        robot.x += v * std::cos(robot.theta) * dt;
        robot.y += v * std::sin(robot.theta) * dt;
        robot.theta += w * dt;
    }

    logFile.close();
    std::cout << "90-degree corner data saved to corner_visualization.csv\n";
}


void test_loop() {
    test_circle_tracking();
    simulate_figure8();
    simulate_90_degree_corner();
}


#endif