#include "Controls.hpp"
#include "LinAlg.hpp"

void Controls::step_50hz() {
    // Assume we somehow get the target state given to us (interfacing to do later)
    target_state = Vec3(); // [v, a_x, w] where a_x is unused in control path
    target_state[0] = 1;

    // Get latest state estimation

    // Individual PID loops
    float u_v = pid_calculate(target_state[0], 
                              estimated_state[0], 
                              kpv, 
                              kiv, 
                              accum_error_v, 
                              0.02); // 0.02s = 50hz

    float u_w = pid_calculate(target_state[2], 
                              estimated_state[2], 
                              kpw, 
                              kiw, 
                              accum_error_w, 
                              0.02); // 0.02s = 50hz

    std::pair<float, float> control_bldc_output = inverse_kinematics(u_v, u_w);

    // Send left and right commands
    float u_r = control_bldc_output.first;
    float u_l = control_bldc_output.second;

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
std::pair<float, float> Controls::forward_kinematics(float w_r, float w_l) {
    // Implements FK mapping from wheel rot speed -> local-frame linear/angular velocities
    float v = WHEEL_RADIUS_MM / 2 * w_r + WHEEL_RADIUS_MM / 2 * w_l;
    float w = WHEEL_RADIUS_MM / LENGTH_WHEEL_TO_WHEEL * w_r - WHEEL_RADIUS_MM / LENGTH_WHEEL_TO_WHEEL * w_l;

    return std::pair<float, float>{v, w};
}

std::pair<float, float> Controls::inverse_kinematics(float v, float w) {
    // Implements IK mapping from local-frame linear/angular velocities -> wheel rot speed
    float w_r = 1 / WHEEL_RADIUS_MM * v + 1 / WHEEL_RADIUS_MM * LENGTH_WHEEL_TO_WHEEL / 2 * w; 
    float w_l = 1 / WHEEL_RADIUS_MM * v - 1 / WHEEL_RADIUS_MM * LENGTH_WHEEL_TO_WHEEL / 2 * w; 

    return std::pair<float, float>{w_r, w_l};
}