#include "RobotController.hpp"

RobotController::RobotController(Servo& servo, int pin): _servo(servo) {
    _servo.attach(pin);
}
void RobotController::openClaw() {
    _servo.write(179);
}

void RobotController::closeClaw() {
    _servo.write(152);
}