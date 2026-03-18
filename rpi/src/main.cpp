#include "AutonomyFSM.hpp"

#include "Perception.hpp"
#include "PurePursuit.hpp"
#include "RescueController.hpp"
#include "Serial.hpp"

#include <cerrno>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>
#include <termios.h>



int main (int argc, char** argv) {
    double pp_lookahead_dist = 1.0;
    double pp_lookahead_tol = 0.2;
    double pp_k_curve = 1.0;
    double pp_k_velocity = 1.0;
    double pp_max_linear_vel = 2.0;
    std::string camera_device = "/dev/video0";
    std::string serial_device = "/dev/ttyACM0";
    int baud_int = 115200;

        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return 0;
        }

        auto need_value = [&](const std::string& flag) -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << flag << "\n";
                print_usage(argv[0]);
                std::exit(2);
            }
            return std::string(argv[++i]);
        };

        if (arg == "--pp-lookahead-dist") {
            const std::string v = need_value(arg);
            if (!parse_double(v, &pp_lookahead_dist)) {
                std::cerr << "Invalid double for " << arg << ": " << v << "\n";
                return 2;
            }
        } else if (arg == "--pp-lookahead-tol") {
            const std::string v = need_value(arg);
            if (!parse_double(v, &pp_lookahead_tol)) {
                std::cerr << "Invalid double for " << arg << ": " << v << "\n";
                return 2;
            }
        } else if (arg == "--pp-k-curve") {
            const std::string v = need_value(arg);
            if (!parse_double(v, &pp_k_curve)) {
                std::cerr << "Invalid double for " << arg << ": " << v << "\n";
                return 2;
            }
        } else if (arg == "--pp-k-velocity") {
            const std::string v = need_value(arg);
            if (!parse_double(v, &pp_k_velocity)) {
                std::cerr << "Invalid double for " << arg << ": " << v << "\n";
                return 2;
            }
        } else if (arg == "--pp-max-linear-vel") {
            const std::string v = need_value(arg);
            if (!parse_double(v, &pp_max_linear_vel)) {
                std::cerr << "Invalid double for " << arg << ": " << v << "\n";
                return 2;
            }
        } else if (arg == "--camera") {
            camera_device = need_value(arg);
        } else if (arg == "--serial") {
            serial_device = need_value(arg);
        } else if (arg == "--baud") {
            const std::string v = need_value(arg);
            if (!parse_int(v, &baud_int)) {
                std::cerr << "Invalid int for " << arg << ": " << v << "\n";
                return 2;
            }
        } else if (arg == "--loop-sleep-us") {
            const std::string v = need_value(arg);
            if (!parse_int(v, &loop_sleep_us)) {
                std::cerr << "Invalid int for " << arg << ": " << v << "\n";
                return 2;
            }
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            print_usage(argv[0]);
            return 2;
        }

    const speed_t baudrate = baud_to_termios(baud_int);
    if (baudrate == 0) {
        std::cerr << "Unsupported baudrate: " << baud_int << "\n";
        return 2;
    }

    PurePursuit pure_pursuit(
        pp_lookahead_dist,
        pp_lookahead_tol,
        pp_k_curve,
        pp_k_velocity,
        pp_max_linear_vel
    );

    Perception perception(camera_device);
    RescueController rescue_controller;
    Serial serial(serial_device, baudrate);

    AutonomyFSM fsm(&pure_pursuit, &perception, &rescue_controller, &serial);

    while (true) {
        fsm.step();
        usleep(loop_sleep_us);
    }
    return 0;
}