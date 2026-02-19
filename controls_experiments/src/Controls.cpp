#include "Controls.hpp"

void Controls::step_50hz() {
    return;
}

float Controls::pid_calculate(float target, float current, float kp, float ki, float &accum_error, float dt) {
    float error = target - current;
    accum_error += error;

    float u = error*kp + accum_error*ki*dt;

    return u;
}

// Returns [pred state, pred covariance]
std::pair<Vec3, Mat3> Controls::predict_step(Mat3 A, Vec3 prev_state, Mat3 prev_P, Mat3 Q) {
    Vec3 pred_state = A * prev_state;
    Mat3 pred_P = A * prev_P * A.transpose() + Q;

    return std::pair<Vec3, Mat3>{pred_state, pred_P};
}

// Returns [corrected state, corrected covariance]
std::pair<Vec3, Mat3> Controls::correct_step(Vec3 z_sensor, Mat3 A, Mat3 H, Mat3 R, Mat3 Q, Vec3 pred_state, Mat3 pred_P) {
    Vec3 innovation = z_sensor - H * pred_state;
    Mat3 S = H * pred_P * H.transpose() + R;

    Mat3 K = pred_P * H.transpose() * S.inverse();

    Vec3 corrected_state = pred_state + K * innovation;
    Mat3 corrected_covar = (Mat3::Identity() - K * H) * pred_P;

    return std::pair<Vec3, Mat3>{corrected_state, corrected_covar};
}

// Kinematics
std::pair<float, float> forward_kinematics(float w_r, float w_l);
std::pair<float, float> inverse_kinematics(float v, float w);