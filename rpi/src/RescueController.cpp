#include "RescueController.hpp"

RescueController::RescueController() {}

bool RescueController::step_grab(Serial& serial) {
    return serial.writeData(CMD_CLAW_CLOSE, {}) > 0;
}

bool RescueController::step_drop(Serial& serial) {
    return serial.writeData(CMD_CLAW_OPEN, {}) > 0;
}
