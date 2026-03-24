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
    uint32_t _last_enc_time = 0
    float r_wheel = 31;
    float max_linear_velocity = 1000; // mm/s
    float max_angular_velocity = max_linear_velocity / r_wheel; // rad/s

    EncoderAS5600 _left_encoder;
    EncoderAS5600 _right_encoder;
    IMUGY61 _imu;
    Controls _controls;

public: 
    RobotController(EncoderAS5600& left_encoder, EncoderAS5600& right_encoder, 
        IMUGY61& imu, Servo& servo, int servo_pin, int motor_L_high_pin, int motor_L_low_pin, int motor_R_high_pin, int motor_R_low_pin);

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
