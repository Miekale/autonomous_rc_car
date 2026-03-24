#include "RobotController.hpp"
#include "Arduino.h"

// RobotController.cpp
RobotController::    RobotController(EncoderAS5600& left_encoder, EncoderAS5600& right_encoder, 
        IMUGY61& imu, Servo& servo, int servo_pin, int motor_L_high_pin, int motor_L_low_pin, 
        int motor_R_high_pin, int motor_R_low_pin)
    : _left_encoder(left_encoder), _right_encoder(right_encoder), _imu(imu), _servo(servo), 
    _m_L_high_pin(motor_L_high_pin), _m_L_low_pin(motor_L_low_pin), _m_R_high_pin(motor_R_high_pin), 
    _m_R_low_pin(motor_R_low_pin) 
{
    pinMode(_m_L_high_pin, OUTPUT);
    pinMode(_m_L_low_pin, OUTPUT);
    pinMode(_m_R_high_pin, OUTPUT);
    pinMode(_m_R_low_pin, OUTPUT);
    _controls = Controls();
    _last_enc_time = millis();
}

void RobotController::openClaw() {
    _servo.write(179);
}

void RobotController::closeClaw() {
    _servo.write(152);
}

void RobotController::init_motors() {
    Serial.println("Initialized Motors");
}

void RobotController::set_m_l_speed(float percent) {
    if (percent < -1 || percent > 1) {
        Serial.println("SPEED (PERCENT) MUST BE BETWEEN 0-1");
        return;
    }
    
    if (percent >= 0) {
        analogWrite(_m_L_high_pin, percent * 155 + 100);  // PWM 255 is max, 0 is min
        analogWrite(_m_L_low_pin, 0);
    }
    else {
        analogWrite(_m_L_low_pin, abs(percent) * 155 + 100);
        analogWrite(_m_L_high_pin, 0);
    }

    Serial.print("LEFT: ");
    Serial.println(percent * 100);
}

void RobotController::set_m_r_speed(float percent) {
    if (percent < -1 || percent > 1) {
        Serial.println("SPEED (PERCENT) MUST BE BETWEEN 0-1");
        return;
    }
    
    if (percent >= 0) {
        analogWrite(_m_R_high_pin, percent * 115 + 100);  // PWM 255 is max, 0 is min
        analogWrite(_m_R_low_pin, 0);
    }
    else {
        analogWrite(_m_R_low_pin, abs(percent) * 115 + 100);
        analogWrite(_m_R_high_pin, 0);
    }

    Serial.print("RIGHT: ");
    Serial.println(percent * 100);
}

void RobotController::execute_v_w_command(float v, float w) {
    // If v > 0 then start running both
<<<<<<< Updated upstream
    float vl = 0;
    float vr = 0;
    Serial.print("v: ");
    Serial.print(v);
    Serial.print("w: ");
    Serial.print(w);

    linalg::FloatPair pair = Controls::inverse_kinematics(v, w);
=======
    float a_x = _imu.getX();

    uint32_t now = millis();
    float dt = (now - _last_enc_time) / 1000.0f;
    _last_enc_time = now;

    _left_encoder.update(dt);
    _right_encoder.update(dt);

    float w_l = _left_encoder.getVelocity();   // or getTicks() / dt
    float w_r = _right_encoder.getVelocity();

    linalg::FloatPair pair = _controls.step_50hz(a_x, w_l, w_r, v, w);
>>>>>>> Stashed changes

    // angular velocities
    vl = pair.first / max_angular_velocity;
    vr = pair.second / max_angular_velocity;

<<<<<<< Updated upstream
    Serial.print("vl: ");
    Serial.print(vl);
    Serial.print("vr: ");
    Serial.println(vr);

    set_m_l_speed(vl);
    set_m_r_speed(vr);
=======
    set_m_l_speed(wl);
    set_m_r_speed(wr); // TODO: this is left compensation
>>>>>>> Stashed changes
}
