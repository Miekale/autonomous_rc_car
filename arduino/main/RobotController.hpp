#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <Servo.h>

class RobotController {
private:
    Servo& _servo;

public: 
    RobotController(Servo& servo, int pin);
    void openClaw();
    void closeClaw();
};

#endif