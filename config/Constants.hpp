#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

struct Position {
    double x, y, theta;
};

const double BULLSEYE_DISTANCE_STOP_LF = 0.1;  // 0.1m 
const double GOAL_DISTANCE_STOP_LF = 0.1;  // 0.1m 

const double CAMERA_INTRINSIC_MATRIX[3][3] = {
    {1157.9275607093223, 0.0, 939.7984863138656},
    {0.0, 801.9018321969417, -47.69928428125636},
    {0.0, 0.0, 1.0},
};

const double CAMERA_DISTORTION_COEFFICIENTS[5] = {
    -0.030313707699100377,
    -0.18557170771313378,
    0.08433296336108598,
    -0.0010328246782788921,
    0.0641909671797407,
};

const double MOUNTING_HEIGHT = 0.105; // 0.1m

const double RED_LOWER[3] = {0, 100, 70};
const double RED_UPPER[3] = {10, 255, 255};

#endif