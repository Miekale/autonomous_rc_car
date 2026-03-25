#include "Controls.hpp"
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
    Serial.println(']');
}
}  // namespace

Controls::Controls() {
    last_time = millis();
    target_state = Vec3();
    estimated_state = Vec3();
}

FloatPair Controls::step(float a_x, float w_l_enc, float w_r_enc, float pps_vl, float pps_w) {
    sensor[0] = w_l_enc;   // raw rad/s from left encoder
    sensor[2] = w_r_enc;   // raw rad/s from right encoder

    // save time and time delta
    uint32_t now = millis();
    time_delta   = now - last_time;
    last_time    = now;
    
    A.idx(0, 1) = time_delta;
    // KF 
    auto [pred_state, pred_P] = predict_step(A, estimated_state, P_covariance, Q_enc, time_delta, a_x);
    auto [est_state, est_P]   = correct_step(sensor, H_enc, R_enc, pred_state, pred_P);

    Serial.print(F("est v: ")); Serial.print(est_state[0]);
    Serial.print(F(" est a: ")); Serial.print(est_state[1]);
    Serial.print(F(" est w: ")); Serial.println(est_state[2]);
    Serial.println("P: ");
    serial_print_mat3(est_P);

    // Update before next time step
    estimated_state = est_state;
    P_covariance    = est_P;

    // reset taget
    target_state[0] = pps_vl;
    target_state[2] = pps_w;

    auto [w_l_target, w_r_target ] = inverse_kinematics(pps_vl, pps_w);

    Serial.println("right PID ------");
    float w_l_pwm = pid_calculate(
        w_l_target, 
        w_l_enc,
        kp_left, 
        ki_left,
        accum_error_left,
        time_delta);

    Serial.println("right PID ------");
    float w_r_pwm = pid_calculate(
        w_r_target, 
        w_r_enc,
        kp_right, 
        ki_right,
        accum_error_right,
        time_delta);

    float left_ff = ff(w_l_target, false);
    float right_ff = ff(w_r_target, true);

    w_l_pwm += left_ff;
    w_r_pwm += right_ff;
    
    return {w_l_pwm, w_r_pwm};
}

float Controls::pid_calculate(float target, float current, float kp, float ki, float &accum_error, float dt) {
    float error = target - current;
    accum_error += error;

    Serial.print("Error: ");
    Serial.println(error);

    float u = error*kp + accum_error*ki*dt;

    return u;
}

float Controls::ff(float target_angular_speed, bool is_right) {
    // PWM 65: 55rad/s
    // PWM 60: 48rad/s
    // PWM 55: 40rad/s
    // PWM 50: 35rad/s
    // PWM 45: 28rad/s
    // PWM 40: 23rad/s
    // PWM 35: 16rad/s
    // fit with excel: ff = 0.78x + 22.7 
    if (is_right) {
        return (target_angular_speed + 20) *0.78 + 22.7;
    }
    return target_angular_speed * 0.78 + 22.7;
}

// Returns [pred state, pred covariance]
StatePair Controls::predict_step(Mat3 A, Vec3 prev_state, Mat3 prev_P, Mat3 Q, uint32_t time_delta, float a_x) {
    Vec3 pred_state = A * prev_state;

    pred_state[0] += (time_delta / 1000.0f) * a_x;
    pred_state[1] = a_x;

    Mat3 pred_P = A * prev_P * A.transpose() + Q;

    return StatePair{pred_state, pred_P};
}

// Returns [corrected state, corrected covariance]
StatePair Controls::correct_step(Vec3 z_sensor, Mat3 H, Mat3 R, Vec3 pred_state, Mat3 pred_P) {
    // Serial.print(F("H * pred_state: "));
    // _vec3(H * pred_state);
    // Serial.println();

    Vec3 innovation = z_sensor - H * pred_state;
    Serial.print(F("Innovation: "));
    serial_print_vec3(innovation);
    Serial.println();

    Mat3 S = H * pred_P * H.transpose() + R;
    //Serial.println(F("Innovation covar:"));
    //serial_print_mat3(S);
    //Serial.println();

    Mat3 K = pred_P * H.transpose() * S.inverse();
    //Serial.println(F("Kalman Gain:"));
    //serial_print_mat3(K);
    //Serial.println();

    //Serial.print(F("Correcting predicted state by: "));
    //serial_print_vec3(K * innovation);
    //Serial.println();
    Vec3 corrected_state = pred_state + K * innovation;
    Mat3 IKH = Mat3::Identity() - K * H;

    // Force symmetry in-case floating point ops cooks it
    Mat3 corrected_covar = IKH * pred_P * IKH.transpose() + K * R * K.transpose();
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

// so first step innovation doesnt blow up
void Controls::init(float w_l, float w_r) {
    FloatPair speed = forward_kinematics(w_l, w_r);
    estimated_state[0] = speed.first;
    estimated_state[1] = 0.0f;
    estimated_state[2] = speed.second;
    last_time = millis();
}