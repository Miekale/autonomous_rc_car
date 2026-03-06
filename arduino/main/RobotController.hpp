#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <Servo.h>

class RobotController {
private:
    Servo& _servo;
    Servo& _motor_l;
    Servo& _motor_r;

public: 
    RobotController(Servo& servo, int servo_pin, Servo& motor_left, int motor_l_pin, Servo& motor_right, int motor_r_pin);

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