#include "RescueController.hpp"

RescueController::RescueController() {}

bool RescueController::step_grab(Serial& serial, double timestamp) {
    return serial.sendWithRetry(CMD_CLAW_CLOSE, {}, timestamp);
}

bool RescueController::step_drop(Serial& serial, double timestamp) {
    return serial.sendWithRetry(CMD_CLAW_OPEN, {}, timestamp);
}

bool RescueController::step_pursuit(Serial& serial, const std::pair<float, float>& ppdata, double timestamp) {
    return serial.writeData(CMD_PURE_PURSUIT, {ppdata.first, ppdata.second}, timestamp);
}
