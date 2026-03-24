#ifndef CONTROL_HPP
#define CONTROL_HPP

#include "LinAlg.hpp"
#include "Arduino.h" 

using linalg::Mat3;
using linalg::Vec3;
using linalg::StatePair;
using linalg::FloatPair;

class Controls {
public:

    static const float WHEEL_RADIUS_MM = 62.0 / 2;
    static const float LENGTH_WHEEL_TO_WHEEL = 160.0;

    // 0.02 comes from estimated 50hz
    Mat3 A = Mat3(
        0.80f, 0.0, 0.0f,
        0.0f,  0.90f, 0.0f,
        0.0f,  0.0f,  0.80f
    );

    const Mat3 R_enc = Mat3(
        50.0f, 0.0f,    0.0f,
        0.0f,    50.0f, 0.0f,
        0.0f,    0.0f,    50.0f
    );

    const Mat3 Q_enc = Mat3(
        10.0f, 0.0f, 0.0f,
        0.0f, 50.0f, 0.0f,
        0.0f, 0.0f, 10.0f
    );

    Mat3 P_covariance = Mat3(
        100.0f, 0.0f, 0.0f, 
        0.0f, 100.0f, 0.0f,
        0.0f, 0.0f, 100.0f
    );

    const Mat3 H_enc = Mat3(
        1.0f / WHEEL_RADIUS_MM, 0.0f, 0.5f * LENGTH_WHEEL_TO_WHEEL / WHEEL_RADIUS_MM,
        0.0f,                   0.0f, 0.0f,
        1.0f / WHEEL_RADIUS_MM, 0.0f, -0.5f * LENGTH_WHEEL_TO_WHEEL / WHEEL_RADIUS_MM
    );

    Controls();

    FloatPair step(float a_x, float w_l_enc, float w_r_enc, float pps_vl, float pps_w);

    uint32_t last_time;
    uint32_t time_delta;
    Vec3 sensor;
    
    Vec3 target_state;

    float kpv;
    float kiv;
    float accum_error_v;

    float kpw;
    float kiw;
    float accum_error_w;

    float pid_calculate(float target, float current, float kp, float ki, float &accum_error, float dt);

    Vec3 estimated_state;

    StatePair predict_step(Mat3 A, Vec3 prev_state, Mat3 prev_P, Mat3 Q, uint32_t time_delta, float a_x);
    StatePair correct_step(Vec3 z_sensor, Mat3 H, Mat3 R, Vec3 pred_state, Mat3 pred_P);

    static FloatPair forward_kinematics(float w_r, float w_l);
    static FloatPair inverse_kinematics(float v, float w);

    void init(float w_l, float w_r);
};

#endif
