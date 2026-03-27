#include "Perception.hpp"
#include "RescueController.hpp"
#include "AutonomyFSM.hpp"
#include "PurePursuit.hpp"
#include "RescueController.hpp"
#include "Serial.hpp"
#include "Constants.hpp"

#include <thread>
#include <iostream>
#include <fstream>

bool debug = true;

int main(int argc, char* argv[]) {
    Serial serial("/dev/ttyACM1", SERIAL_BAUD_RATE);
    PurePursuit pure_pursuit(
        LOOK_AHEAD_DISTANCE,
        LOOK_AHEAD_TOL,
        K_CURVE,
        K_VELOCITY,
        MAX_LINEAR_VELOCITY
    );
    RescueController rescue_controller;

    Perception perception = argc > 1
        ? Perception(argv[1], true, debug, true)
        : Perception("0", false, debug, true);

    // std::ifstream f(argv[1]);
    // if (!f.is_open()) {
    //     std::cerr << "Could not open file: " << argv[1] << std::endl;
    //     return 1;
    // }

    AutonomyFSM fsm(&pure_pursuit, &perception, &rescue_controller, &serial, debug);

    std::cout << "DONE INI" << std::endl;
    std::cout << "Stepping FSM..." << std::endl;
    while (true) {
        auto start = std::chrono::steady_clock::now();

        fsm.step();

        auto elapsed = std::chrono::steady_clock::now() - start;
        auto sleep_for = PERIOD - elapsed;

        if (sleep_for > std::chrono::duration<double>(0)) {
           std::this_thread::sleep_for(sleep_for);
        }
    }
}