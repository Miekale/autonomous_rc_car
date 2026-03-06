#include "RobotController.hpp"
#include "Arduino.h"

// RobotController.cpp
RobotController::RobotController(Servo& servo, int servo_pin, Servo& motor_left, int motor_l_pin, Servo& motor_right, int motor_r_pin)
    : _servo(servo), _motor_l(motor_left), _motor_r(motor_right)  // initializer list for references
{
    _servo.attach(servo_pin);
    _motor_l.attach(motor_l_pin);
    _motor_r.attach(motor_r_pin);
}

void RobotController::openClaw() {
    _servo.write(179);
}

void RobotController::closeClaw() {
    _servo.write(152);
}

void RobotController::init_motors() {
  Serial.println("Sending 1000 PWM for 15 seconds: plug in ESCs left and right...");

  _motor_l.writeMicroseconds(1000);
  _motor_r.writeMicroseconds(1000);

  delay(15000);

  Serial.println("1000 handshake period over! If 3 rising tones then ARMED");
}

void RobotController::set_m_l_speed(float percent) {
    if (percent < 0 || percent > 1) {
        Serial.println("SPEED (PERCENT) MUST BE BETWEEN 0-1");
        return false;
    }

    _motor_l.writeMicroseconds(1000 + percent * 1000);
    Serial.print("LEFT: ");
    Serial.println(1000 + percent * 1000);}

void RobotController::set_m_r_speed(float percent) {
    if (percent < 0 || percent > 1) {
        Serial.println("SPEED (PERCENT) MUST BE BETWEEN 0-1");
        return;
    }

    _motor_r.writeMicroseconds(1000 + percent * 1000);
    Serial.print("RIGHT: ");
    Serial.println(1000 + percent * 1000);
}

void RobotController::execute_v_w_command(float v, float w) {
    // Stupid Implementation for now

    // If v > 0 then start running both
    float vl = 0;
    float vr = 0;

    if (v > 0) {
        vl += 0.8;
        vr += 0.8;
    }

    // If w > 0, make left faster
    if (w > 0) {
        vr -= 0.2;
    }

    // If w < 0, make right faster
    if (w < 0) {
        vl -= 0.2;
    }

    set_m_l_speed(vl);
    set_m_r_speed(vr);
}
