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
    v_target = 0;
    w_target = 0;
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
    analogWrite(_m_L_en_pin,  abs(percent * LEFT_COMPENSATION) * 50 + 30);


    Serial.print("LEFT: ");
    Serial.println(abs(percent) * 50 + 30);

    /*
    
    10: 0
    20: 0
    30: 30
    40: 

    
    */
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
    analogWrite(_m_R_en_pin, abs(percent) * 50 + 30);

    Serial.print("RIGHT: ");
    Serial.println(abs(percent) * 50 + 3);
}


void RobotController::update_v_w_command(float v, float w) {
    v_target = v;
    w_target = w;
}

void RobotController::execute_v_w_command(float dt) {
    float a_x = _imu.getX();
    //Serial.print("IMU a_x: "); Serial.println(a_x, 5);

    //Serial.print(F("dt: ")); Serial.println(dt, 5);  // add here

    _left_encoder.update(dt);
    _right_encoder.update(dt);

    //Serial.print("left encoder ticks: ");Serial.println(_left_encoder.getTicks());
    //Serial.print("right encoder ticks: ");Serial.println(_right_encoder.getTicks());

    float w_l = _left_encoder.getVelocity();   // rad/s
    float w_r = _right_encoder.getVelocity();

    Serial.print("Encoder readings: ");Serial.print(w_l); Serial.print(" "); Serial.println(w_r);

    linalg::FloatPair command = _controls.step(a_x, w_l, w_r, v_target, w_target, dt);
    float left_wheel_pwm = command.first;
    float right_wheel_pwm = command.second;

    Serial.print("Executing desired PWM speeds for l and r: "); Serial.print(command.first, 4); Serial.print(" "); Serial.println(command.second, 4);
    Serial.print("For desired speeds of wl, and wr: "); Serial.print(w_l, 4); Serial.print(" "); Serial.println(w_r, 4);


    if (left_wheel_pwm > 0) {
        left_wheel_pwm = min(left_wheel_pwm, 255);
        digitalWrite(_m_L_high_pin, HIGH);
        digitalWrite(_m_L_low_pin, LOW);
        analogWrite(_m_L_en_pin, left_wheel_pwm);
    } else {
        left_wheel_pwm = max(left_wheel_pwm, -255);
        digitalWrite(_m_L_high_pin, LOW);
        digitalWrite(_m_L_low_pin, HIGH);
        analogWrite(_m_L_en_pin, -left_wheel_pwm);
    }
    
    if (right_wheel_pwm >0 ) {
        right_wheel_pwm = min(right_wheel_pwm, 255);
        digitalWrite(_m_R_high_pin, HIGH);
        digitalWrite(_m_R_low_pin, LOW);
        analogWrite(_m_R_en_pin, right_wheel_pwm);
    } else {
        right_wheel_pwm = max(right_wheel_pwm, -255);
        digitalWrite(_m_R_high_pin, LOW);
        digitalWrite(_m_R_low_pin, HIGH);
        analogWrite(_m_R_en_pin, -right_wheel_pwm);
    }

    // Control loop speed sanity checker
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
