#include "RobotController.hpp"
#include "Arduino.h"

// RobotController.cpp
RobotController::    RobotController(Servo& servo, int servo_pin, int motor_L_high_pin, int motor_L_low_pin, int motor_R_high_pin, int motor_R_low_pin)
    : _servo(servo), _m_L_high_pin(motor_L_high_pin), _m_L_low_pin(motor_L_low_pin), _m_R_high_pin(motor_R_high_pin), _m_R_low_pin(motor_R_low_pin) 
{
    pinMode(_m_L_high_pin, OUTPUT);
    pinMode(_m_L_low_pin, OUTPUT);
    pinMode(_m_R_high_pin, OUTPUT);
    pinMode(_m_R_low_pin, OUTPUT);
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
        analogWrite(_m_L_high_pin, percent * 255);  // PWM 255 is max, 0 is min
        analogWrite(_m_L_low_pin, 0);
    }
    else {
        analogWrite(_m_L_low_pin, abs(percent) * 255);
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
        analogWrite(_m_R_high_pin, percent * 255);  // PWM 255 is max, 0 is min
        analogWrite(_m_R_low_pin, 0);
    }
    else {
        analogWrite(_m_R_low_pin, abs(percent) * 255);
        analogWrite(_m_R_high_pin, 0);
    }

    Serial.print("RIGHT: ");
    Serial.println(percent * 100);
}

void RobotController::execute_v_w_command(float v, float w) {
    // Stupid Implementation for now

    // If v > 0 then start running both
    float vl = 0;
    float vr = 0;

    linalg::FloatPair pair = Controls::inverse_kinematics(v, w);

    vl = pair.first / 0.2; // TODO put the 0.2 into a constants file, it's the max linear speed for now
    vr = pair.second / 0.2;

    Serial.print("vl: ")
    Serial.print(vl)
    Serial.print("vr: ")
    Serial.println(vr)

    set_m_l_speed(vl);
    set_m_r_speed(vr);
}
