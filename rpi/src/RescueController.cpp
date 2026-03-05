#include "RescueController.hpp"
#include <iostream>
RescueController::RescueController() {}

bool RescueController::step_grab(Serial& serial) {
    return serial.sendWithRetry(CMD_CLAW_CLOSE, {});
}

bool RescueController::step_drop(Serial& serial) {
    return serial.sendWithRetry(CMD_CLAW_OPEN, {});
}
