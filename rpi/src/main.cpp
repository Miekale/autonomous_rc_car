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
#include <unordered_map>

void print_usage(const char* prog) {
    std::cerr
        << "Usage: " << prog << " [options]\n"
        << "\nOptions:\n"
        << "  -h, --help                 Show this help message\n"
        << "  --pp-lookahead-dist <f>    Pure pursuit lookahead distance (default: 1.0)\n"
        << "  --pp-lookahead-tol  <f>    Pure pursuit lookahead tolerance (default: 0.2)\n"
        << "  --pp-k-curve        <f>    Pure pursuit curvature gain      (default: 1.0)\n"
        << "  --pp-k-velocity     <f>    Pure pursuit velocity gain       (default: 1.0)\n"
        << "  --pp-max-linear-vel <f>    Pure pursuit max linear velocity (default: 2.0)\n"
        << "  --camera            <dev>  Camera device path   (default: /dev/video0)\n"
        << "  --video             <bool> If camera is video file (default: false)\n"
        << "  --serial            <dev>  Serial device path   (default: /dev/ttyACM0)\n"
        << "  --baud              <int>  Serial baud rate     (default: 115200)\n"
        << "  --loop-sleep-us     <int>  Main loop sleep (us) (default: 10000)\n";
}

bool parse_double(const std::string& s, double* out) {
    try {
        size_t pos;
        *out = std::stod(s, &pos);
        return pos == s.size(); // false if trailing non-numeric chars
    } catch (const std::exception&) {
        return false;
    }
}

bool parse_int(const std::string& s, int* out) {
    try {
        size_t pos;
        *out = std::stoi(s, &pos);
        return pos == s.size();
    } catch (const std::exception&) {
        return false;
    }
}


bool parse_bool(const std::string& s, bool* out) {
    try {
        size_t pos;
        *out = std::stoi(s, &pos);
        return pos == s.size();
    } catch (const std::exception&) {
        return false;
    }
}

speed_t baud_to_termios(int baud) {
    static const std::unordered_map<int, speed_t> table = {
        {50,      B50},
        {75,      B75},
        {110,     B110},
        {134,     B134},
        {150,     B150},
        {200,     B200},
        {300,     B300},
        {600,     B600},
        {1200,    B1200},
        {1800,    B1800},
        {2400,    B2400},
        {4800,    B4800},
        {9600,    B9600},
        {19200,   B19200},
        {38400,   B38400},
        {57600,   B57600},
        {115200,  B115200},
        {230400,  B230400},
        {460800,  B460800},
        {500000,  B500000},
        {576000,  B576000},
        {921600,  B921600},
        {1000000, B1000000},
        {1152000, B1152000},
        {1500000, B1500000},
        {2000000, B2000000},
        {2500000, B2500000},
        {3000000, B3000000},
        {3500000, B3500000},
        {4000000, B4000000},
    };
    auto it = table.find(baud);
    return (it != table.end()) ? it->second : 0; // 0 = unsupported
}


int main(int argc, char** argv) {
    double pp_lookahead_dist  = 1.0;
    double pp_lookahead_tol   = 0.2;
    double pp_k_curve         = 1.0;
    double pp_k_velocity      = 1.0;
    double pp_max_linear_vel  = 2.0;
    std::string camera_device = "0";
    bool video_file = false;
    std::string serial_device = "/dev/ttyACM0";
    int baud_int              = 115200;
    int loop_sleep_us         = 10000; // default: 10 ms

    // Lambda must be defined before the loop but captures 'i' by reference,
    // so declare 'i' here and let the lambda close over it.
    int i = 1;
    auto need_value = [&](const std::string& flag) -> std::string {
        if (i + 1 >= argc) {
            std::cerr << "Missing value for " << flag << "\n";
            print_usage(argv[0]);
            std::exit(2);
        }
        return std::string(argv[++i]);
    };

    for (; i < argc; ++i) {
        const std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return 0;
        } else if (arg == "--pp-lookahead-dist") {
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
            const std::string v = need_value(arg);
            if (!parse_int(v, &camera_device)) {
                std::cerr << "Invalid int for " << arg << ": " << v << "\n";
                return 2;
            }
        } else if (arg == "--video") {
            const std::string v = need_value(arg);
            if (!parse_bool(v, &video_file)) {
                std::cerr << "Invalid bool for " << arg << ": " << v << "\n";
                return 2;
            }
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
    }

    // Validate and convert baudrate after all args are parsed
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

    Perception        perception(camera_device, is_video);
    RescueController  rescue_controller;
    Serial            serial(serial_device, baudrate);
    AutonomyFSM       fsm(&pure_pursuit, &perception, &rescue_controller, &serial);

    while (true) {
        fsm.step();
        usleep(loop_sleep_us);
    }

    return 0;
}