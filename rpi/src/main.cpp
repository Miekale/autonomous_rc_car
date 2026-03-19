#include "Perception.hpp"
#include "RescueController.hpp"
#include "AutonomyFSM.hpp"
#include "PurePursuit.hpp"
#include "RescueController.hpp"
#include "Serial.hpp"
#include "Constants.hpp"

#include <iostream>

int main() {
    Serial mySerial("/dev/ttyACM0", SERIAL_BAUD_RATE);

    PurePursuit pure_pursuit(
        LOOK_AHEAD_DISTANCE,
        LOOK_AHEAD_TOL,
        K_CURVE,
        K_VELOCITY,
        MAX_LINEAR_VELOCITY
    );
    RescueController rescue_controller;
    Perception perception("0");
    Serial serial(mySerial, SERIAL_BAUD_RATE);
    AutonomyFSM fsm(&pure_pursuit, &perception, &rescue_controller, &serial);
}