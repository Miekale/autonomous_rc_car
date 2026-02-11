#ifndef RESCUE_CONTROLLER_HPP
#define RESCUE_CONTROLLER_HPP

#include "Constants.hpp"

class RescueController {
private:
public:
    RescueController();
    /**
    Steps the Rescue Controller.

    Returns True if rescue is done, False otherwise.
    */
    bool step_grab();
    bool step_drop();
};

#endif