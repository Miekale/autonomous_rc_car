#include "RescueController.hpp"

RescueController::RescueController() {}

bool RescueController::step_grab(Serial& serial) {
    return serial.sendWithRetry(CMD_CLAW_CLOSE, {});
}

bool RescueController::step_drop(Serial& serial) {
    return serial.sendWithRetry(CMD_CLAW_OPEN, {});
}

bool RescueController::step_pursuit(Serial& serial, const std::pair<float, float>& ppdata) {
    return serial.writeData(CMD_PURE_PURSUIT, {ppdata.first, ppdata.second});
}
