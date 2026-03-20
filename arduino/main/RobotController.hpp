#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <Servo.h>
#include "Control.hpp"
#include "LinAlg.hpp"

class RobotController {
private:

    Servo& _servo;
    
    int _m_L_high_pin;
    int _m_L_low_pin;
    int _m_R_high_pin;
    int _m_R_low_pin;

    float r_wheel = 31;
    float max_linear_velocity = 1000; // mm/s
    float max_angular_velocity = max_linear_velocity / r_wheel; // rad/s

public: 
    RobotController(Servo& servo, int servo_pin, int motor_L_high_pin, int motor_L_low_pin, int motor_R_high_pin, int motor_R_low_pin);

    // Claw
    void openClaw();
    void closeClaw();

    // Motors
    void init_motors();
    void execute_v_w_command(float v, float w);
    void set_m_l_speed(float percent);
    void set_m_r_speed(float percent);
};

#endif
