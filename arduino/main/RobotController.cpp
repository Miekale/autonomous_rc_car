#include "RobotController.hpp"
#include "Arduino.h"

// RobotController.cpp
RobotController::RobotController(EncoderAS5600& left_encoder, EncoderAS5600& right_encoder, 
        IMUGY61& imu, Servo& servo, int servo_pin, int motor_L_high_pin, int motor_L_low_pin, 
        int motor_L_en_pin, int motor_R_high_pin, int motor_R_low_pin, int motor_R_en_pin)
    : _left_encoder(left_encoder), _right_encoder(right_encoder), _imu(imu), _servo(servo), 
    _m_L_high_pin(motor_L_high_pin), _m_L_low_pin(motor_L_low_pin), _m_L_en_pin(motor_L_en_pin),
     _m_R_high_pin(motor_R_high_pin), _m_R_low_pin(motor_R_low_pin), _m_R_en_pin(motor_R_en_pin)
{
    pinMode(_m_L_high_pin, OUTPUT);
    pinMode(_m_L_low_pin, OUTPUT);
    pinMode(_m_R_high_pin, OUTPUT);
    pinMode(_m_R_low_pin, OUTPUT);
    pinMode(_m_R_en_pin,   OUTPUT);
    pinMode(_m_L_en_pin,   OUTPUT);
    _last_enc_time = millis();
    _last_time_second = millis();
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
    if (abs(percent) < 0.01f) {
        digitalWrite(_m_L_high_pin, LOW);
        digitalWrite(_m_L_low_pin, LOW);
        analogWrite(_m_L_en_pin, 0);
        return;
    }
    if (percent > 0) {
        digitalWrite(_m_L_high_pin, HIGH);
        digitalWrite(_m_L_low_pin, LOW);
    } else {
        digitalWrite(_m_L_high_pin, LOW);
        digitalWrite(_m_L_low_pin, HIGH);
    }
    analogWrite(_m_L_en_pin, 80);


    Serial.print("LEFT: ");
    Serial.println(abs(percent) * 50 + 30);
}

void RobotController::set_m_r_speed(float percent) {
    if (percent < -1 || percent > 1) {
        Serial.println("SPEED (PERCENT) MUST BE BETWEEN 0-1");
        return;
    }

    if (abs(percent) < 0.01f) {
        digitalWrite(_m_R_high_pin, LOW);
        digitalWrite(_m_R_low_pin, LOW);
        analogWrite(_m_R_en_pin, 0);
        return;
    }

    if (percent > 0) {
        digitalWrite(_m_R_high_pin, HIGH);
        digitalWrite(_m_R_low_pin, LOW);
    } else {
        digitalWrite(_m_R_high_pin, LOW);
        digitalWrite(_m_R_low_pin, HIGH);
    }
    analogWrite(_m_R_en_pin, 80);

    Serial.print("RIGHT: ");
    Serial.println(abs(percent) * 50 + 30);
}

void RobotController::execute_v_w_command(float v, float w) {
    // If v > 0 then start running both
    float a_x = _imu.getX();
    // TODO: re-tune the IMU to get 0 accel when NOT moving
    a_x = 0;
    Serial.print("IMU a_x: "); Serial.println(a_x);

    uint32_t now = millis();
    float dt = (now - _last_enc_time) / 1000.0f;
    _last_enc_time = now;

    Serial.print(F("dt: ")); Serial.println(dt);  // add here

    _left_encoder.update(dt);
    _right_encoder.update(dt);

    float w_l = _left_encoder.getVelocity();   // or getTicks() / dt
    float w_r = _right_encoder.getVelocity();

    Serial.print("Encoder readings: ");Serial.print(w_l); Serial.print(" "); Serial.println(w_r);

    linalg::FloatPair pair = _controls.step(a_x, w_l, w_r, v, w);

    // TODO: this controls implementation doesn't make sense, rewrite it
    // do the PID loop much closer to motors, output velocity error from PID, output PWM directly?
    // angular velocities
    float command_wl = pair.first / max_angular_velocity;
    float command_wr = pair.second / max_angular_velocity;
    Serial.print("Executing left and right desired angular speeds: "); Serial.print(pair.first, 4); Serial.print(" "); Serial.println(pair.second, 4);

    set_m_l_speed(command_wl);
    set_m_r_speed(command_wr); 

    _second_count++;

    if (_second_count >= freq) {
        // One full second of iterations has elapsed
        uint32_t elapsed = millis() - _last_time_second;

        if (elapsed > 1000) {
            Serial.print(F("WARNING: running slow, 50 iterations took "));
            Serial.print(elapsed);
            Serial.println(F("ms"));
        } else {
            delay(1000 - elapsed);
        }

        _second_count = 0;
        _last_time_second = millis();
    }
}
