#ifndef RESCUE_CONTROLLER_HPP
#define RESCUE_CONTROLLER_HPP

#include "Serial.hpp"

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
};

#endif