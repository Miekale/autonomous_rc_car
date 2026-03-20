#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP
#include <vector>
#include <termios.h>
#include <chrono>

struct Position {
    double x, y, theta;
};

const double BULLSEYE_DISTANCE_STOP_LF = 100;
const double GOAL_DISTANCE_STOP_LF = 100;

const float LOOK_AHEAD_DISTANCE = 500;
const float LOOK_AHEAD_TOL = 200; 
const float K_CURVE = 1;
const float K_VELOCITY = 1;
const float MAX_LINEAR_VELOCITY = 200; // mm/s

const speed_t SERIAL_BAUD_RATE = B38400;

const int LOOP_HZ = 30;
const auto PERIOD = std::chrono::duration<double>(1.0 / LOOP_HZ);

// Camera intrinsics
const std::vector<double> DISTORTION_COEFFICIENTS = {
    0.03659055411577084,
    0.08652985034604993,
    0.007419122298149112,
   -0.004124483859329216,
   -0.22428229944263292
};

const std::vector<std::vector<double>> INTRINSIC_MATRIX = {
    { 1092.9386993970447,    0.0,             902.7083365240517 },
    {    0.0,             1090.4539333745056,  793.0601365557278 },
    {    0.0,                0.0,               1.0             }
};

constexpr double MOUNTING_HEIGHT = 96.635;

#endif