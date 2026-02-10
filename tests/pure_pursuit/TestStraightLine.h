#ifndef TEST_STRAIGHT_LINE_H
#define TEST_STRAIGHT_LINE_H

#include "PurePursuit.hpp"
#include "PurePursuitControls.hpp"
#include <iostream>
#include <cassert>

void test_can_travel_basic() {
 std::vector<Position> path = {
        {0,0,0}, {1,0,0}, {2,0,0}, {3,0,0}, {4,0,0}
    };

    Position robot{0, -0.5, 0}; // Robot is below the line y=0

    PurePursuit pp(1.0, 0.2);
    auto lookahead = pp.findLookaheadPoint(robot, path);

    // Assert that we actually found a lookahead point on the path
    assert(lookahead.x > 0);
    assert(lookahead.y == 0);

    PurePursuitControls ctrl(1.0, 1.0, 1.0, 2.0);
    std::pair<double, double> result = ctrl.getControl(robot, lookahead, path);
    double v = result.first;
    double w = result.second;

    // ASSERT: Robot is below path, so it should turn LEFT (positive angular velocity)
    assert(w > 0);

    // Log for plotting: x, y, lookahead_x, lookahead_y, v, w
    std::cout << robot.x << "," << robot.y << "," 
              << lookahead.x << "," << lookahead.y << "," 
              << v << "," << w << std::endl;
    std::cout << "[PASS] test_can_travel_basic\n";
}

void test_lookahead_prefers_heading_alignment() {
    std::vector<Position> path = {
        {1, 0, 0},     // straight ahead (should win)
        {0.7, 0.7, 0}, // 45 deg
        {0.7,-0.7, 0}  // -45 deg
    };

    Position robot{0,0,0};

    PurePursuit pp(1.0, 0.15);
    Position lookahead = pp.findLookaheadPoint(robot, path);

    // Expect closest to heading direction (positive x-axis)
    assert(std::abs(lookahead.x - 1.0) < 1e-6);
    assert(std::abs(lookahead.y) < 1e-6);

    std::cout << "[PASS] test_lookahead_prefers_heading_alignment passed\n";
}

void test_zigzag_tracking_with_tolerance() {
    std::vector<Position> path = {
        {1,0,0},
        {2,1,0},
        {3,0,0},
        {4,1,0},
        {5,0,0}
    };

    Position robot{0,-0.7,0};

    PurePursuit pp(1.0, 0.25);
    PurePursuitControls ctrl(1.0, 1.0, 1.0, 2.0);

    int targetIdx = 0;
    const double reachTol = 0.6;   // important: not too tight
    const double dt = 0.1;

    for (int i = 0; i < 150; i++) {

        Position lookahead = pp.findLookaheadPoint(robot, path);
        auto [v,w] = ctrl.getControl(robot, lookahead, path);

        // --- simulate motion ---
        robot.x += v * cos(robot.theta) * dt;
        robot.y += v * sin(robot.theta) * dt;
        robot.theta += w * dt;

        // --- check waypoint reach ---
        if (targetIdx < path.size()) {
            double dx = robot.x - path[targetIdx].x;
            double dy = robot.y - path[targetIdx].y;
            double dist = std::sqrt(dx*dx + dy*dy);

            if (dist < reachTol) {
                targetIdx++;
            }
        }
    }

    // Assert all waypoints reached
    assert(targetIdx == path.size());

    std::cout << "[PASS] test_zigzag_tracking_with_tolerance passed\n";
}


void test_straight_line() {
    test_can_travel_basic();
    test_lookahead_prefers_heading_alignment();
    test_zigzag_tracking_with_tolerance();
}

#endif