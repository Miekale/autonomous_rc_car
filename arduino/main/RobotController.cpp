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
    if (abs(percent) < 0.01) {
      analogWrite(_m_L_high_pin, 0);
      analogWrite(_m_L_low_pin, 0);
      
    } else if (percent >= 0) {
        analogWrite(_m_L_high_pin, percent * 185 + 70);  // PWM 255 is max, 0 is min
        analogWrite(_m_L_low_pin, 0);
    }
    else {
        analogWrite(_m_L_low_pin, abs(percent) * 185 + 70);
        analogWrite(_m_L_high_pin, 0);
    }

    Serial.print("LEFT: ");
    Serial.println(abs(percent) * 185 + 70);
}

void RobotController::set_m_r_speed(float percent) {
    if (percent < -1 || percent > 1) {
        Serial.println("SPEED (PERCENT) MUST BE BETWEEN 0-1");
        return;
    }
    if (abs(percent) < 0.01) {
      analogWrite(_m_R_high_pin, 0);
      analogWrite(_m_R_low_pin, 0);
    }
    else if (percent >= 0) {
        analogWrite(_m_R_high_pin, percent * 185 + 70);  // PWM 255 is max, 0 is min
        analogWrite(_m_R_low_pin, 0);
    }
    else {
        analogWrite(_m_R_low_pin, abs(percent) * 185 + 70);
        analogWrite(_m_R_high_pin, 0);
    }

    Serial.print("RIGHT: ");
    Serial.println(abs(percent) * 185 + 70);
}

void RobotController::execute_v_w_command(float v, float w) {
    // Stupid Implementation for now

    // If v > 0 then start running both
    float wl = 0;
    float wr = 0;
    Serial.print("v: ");
    Serial.print(v);
    Serial.print(" w: ");
    Serial.println(w);

    linalg::FloatPair pair = Controls::inverse_kinematics(v, -w);

    // angular velocities
    wl = pair.second / max_angular_velocity;
    wr = pair.first / max_angular_velocity;

    Serial.print("vl: ");
    Serial.print(wl);
    Serial.print(" vr: ");
    Serial.println(wr);

    set_m_l_speed(wl);
    set_m_r_speed(wr); // TODO: this is left compensation
}
