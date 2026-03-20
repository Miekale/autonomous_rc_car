#include "Control.hpp"
#include "LinAlg.hpp"

#include <Arduino.h>

namespace {
void serial_print_vec3(const linalg::Vec3& v) {
    Serial.print('[');
    Serial.print(v[0]);
    Serial.print(F(", "));
    Serial.print(v[1]);
    Serial.print(F(", "));
    Serial.print(v[2]);
    Serial.print(']');
}

void serial_print_mat3(const linalg::Mat3& m) {
    Serial.print('[');
    Serial.print(m.idx(0, 0));
    Serial.print(F(", "));
    Serial.print(m.idx(0, 1));
    Serial.print(F(", "));
    Serial.print(m.idx(0, 2));
    Serial.println(']');

    Serial.print('[');
    Serial.print(m.idx(1, 0));
    Serial.print(F(", "));
    Serial.print(m.idx(1, 1));
    Serial.print(F(", "));
    Serial.print(m.idx(1, 2));
    Serial.println(']');

    Serial.print('[');
    Serial.print(m.idx(2, 0));
    Serial.print(F(", "));
    Serial.print(m.idx(2, 1));
    Serial.print(F(", "));
    Serial.print(m.idx(2, 2));
    Serial.print(']');
}
}  // namespace

Controls::Controls() {
    // TODO: Tune these later with car
    kpv = 0.0f;
    kiv = 0.0f;
    kpw = 0.0f;
    kiw = 0.0f;

    accum_error_v = 0.0f;
    accum_error_w = 0.0f;

    target_state = Vec3();
    error_state = Vec3();
    estimated_state = Vec3();
}

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

    FloatPair control_bldc_output = inverse_kinematics(u_v, u_w);

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
StatePair Controls::predict_step(Mat3 A, Vec3 prev_state, Mat3 prev_P, Mat3 Q) {
    Vec3 pred_state = A * prev_state;
    Mat3 pred_P = A * prev_P * A.transpose() + Q;

    return StatePair{pred_state, pred_P};
}

// Returns [corrected state, corrected covariance]
StatePair Controls::correct_step(Vec3 z_sensor, Mat3 H, Mat3 R, Vec3 pred_state, Mat3 pred_P) {
    Serial.print(F("H * pred_state: "));
    serial_print_vec3(H * pred_state);
    Serial.println();

    Vec3 innovation = z_sensor - H * pred_state;
    Serial.print(F("Innovation: "));
    serial_print_vec3(innovation);
    Serial.println();

    Mat3 S = H * pred_P * H.transpose() + R;
    Serial.println(F("Innovation covar:"));
    serial_print_mat3(S);
    Serial.println();

    Mat3 K = pred_P * H.transpose() * S.inverse();
    Serial.println(F("Kalman Gain:"));
    serial_print_mat3(K);
    Serial.println();

    Serial.print(F("Correcting predicted state by: "));
    serial_print_vec3(K * innovation);
    Serial.println();
    Vec3 corrected_state = pred_state + K * innovation;
    Mat3 IKH = Mat3::Identity() - K * H;
    Mat3 corrected_covar = IKH * pred_P * IKH.transpose() + K * R * K.transpose();

    // Force symmetry in-case floating point ops cooks it
    corrected_covar = (corrected_covar + corrected_covar.transpose()) * 0.5f;

    return StatePair{corrected_state, corrected_covar};
}

// Kinematics
FloatPair Controls::forward_kinematics(float w_r, float w_l) {
    // Implements FK mapping from wheel rot speed -> local-frame linear/angular velocities
    float v = WHEEL_RADIUS_MM / 2 * w_r + WHEEL_RADIUS_MM / 2 * w_l;
    float w = WHEEL_RADIUS_MM / LENGTH_WHEEL_TO_WHEEL * w_r - WHEEL_RADIUS_MM / LENGTH_WHEEL_TO_WHEEL * w_l;

    return FloatPair{v, w};
}

FloatPair Controls::inverse_kinematics(float v, float w) {
    // Implements IK mapping from local-frame linear/angular velocities -> wheel rot speed
    float w_r = 1 / WHEEL_RADIUS_MM * v + 1 / WHEEL_RADIUS_MM * LENGTH_WHEEL_TO_WHEEL / 2 * w*3; 
    float w_l = 1 / WHEEL_RADIUS_MM * v - 1 / WHEEL_RADIUS_MM * LENGTH_WHEEL_TO_WHEEL / 2 * w*3; 

    return FloatPair{w_r, w_l};
}
