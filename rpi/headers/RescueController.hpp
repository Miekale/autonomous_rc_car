#ifndef RESCUE_CONTROLLER_HPP
#define RESCUE_CONTROLLER_HPP

#include "Serial.hpp"
#include <iostream>
#include <utility>

class RescueController {
private:
public:
    RescueController();
    /**
    Steps the Rescue Controller.

    Returns True if rescue is done, False otherwise.
    */
    bool step_grab(Serial& serial);
    bool step_drop(Serial& serial);
    bool step_pursuit(Serial& serial, const std::pair<float, float>& ppdata);
};

#endif