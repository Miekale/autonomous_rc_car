#ifndef CONTROLS_HPP
#define CONTROLS_HPP

#include "LinAlg.hpp"
#include <utility>

using namespace linalg;

class Controls {
public:

    const float WHEEL_RADIUS_MM = 62.0 / 2; // mm
    const float LENGTH_WHEEL_TO_WHEEL = 160.0; // mm

    Controls();

    // Runs 1 step
    // Calls FOC libraries directly to set motor speed
    // Assumes this serial comm is fast enough to be non-blocking
    void step_50hz();

private:
    // PID path
    Vec3 target_state;
    Vec3 error_state;

    float kpv;
    float kiv;
    float accum_error_v;

    float kpw;
    float kiw;
    float accum_error_w;

    float pid_calculate(float target, float current, float kp, float ki, float &accum_error, float dt);

    // Estimation path
    Vec3 estimated_state;

    // Returns [pred state, pred covariance]
    std::pair<Vec3, Mat3> predict_step(Mat3 A, Vec3 prev_state, Mat3 prev_P, Mat3 Q);

    // Returns [corrected state, corrected covariance]
    std::pair<Vec3, Mat3> correct_step(Vec3 z_sensor, Mat3 A, Mat3 H, Mat3 R, Mat3 Q, Vec3 prev_state, Mat3 prev_P);

    // Kinematics
    std::pair<float, float> forward_kinematics(float w_r, float w_l);
    std::pair<float, float> inverse_kinematics(float v, float w);
};

#endif 