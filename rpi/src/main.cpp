#include "Perception.hpp"
#include "RescueController.hpp"
#include "AutonomyFSM.hpp"
#include "PurePursuit.hpp"
#include "RescueController.hpp"
#include "Serial.hpp"
#include "Constants.hpp"

#include <thread>
#include <iostream>

int main() {
    Serial serial("/dev/ttyACM0", SERIAL_BAUD_RATE);
    PurePursuit pure_pursuit(
        LOOK_AHEAD_DISTANCE,
        LOOK_AHEAD_TOL,
        K_CURVE,
        K_VELOCITY,
        MAX_LINEAR_VELOCITY
    );
    RescueController rescue_controller;
    Perception perception("0", false, false);
    AutonomyFSM fsm(&pure_pursuit, &perception, &rescue_controller, &serial);

    std::cout << "DONE INI" << std::endl;
    std::cout << "Stepping FSM..." << std::endl;
    while (true) {
        auto start = std::chrono::steady_clock::now();

        serial.writeData(CMD_PURE_PURSUIT, {MAX_LINEAR_VELOCITY / 4, 0}, timestamp);


        auto elapsed = std::chrono::steady_clock::now() - start;
        auto sleep_for = PERIOD - elapsed;

        if (sleep_for > std::chrono::duration<double>(0)) {
            std::this_thread::sleep_for(sleep_for);
        }
    }
}