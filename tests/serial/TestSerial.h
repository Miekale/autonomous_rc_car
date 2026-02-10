#ifndef TEST_SERIAL_H
#define TEST_SERIAL_H

#ifdef __linux__

#include <pty.h>
#include <cstring>
#include <cassert>
#include <iostream>
#include <unistd.h>
#include "Serial.hpp"

void test_serial_pty_basic() {
    int rpi_fd, arduino_fd;
    char arduino_name[128];

    int rc = openpty(&rpi_fd, &arduino_fd, arduino_name, nullptr, nullptr);
    assert(rc == 0);

    Serial serial(arduino_name, B115200);

    const char* msg = "hello serial\n";
    write(rpi_fd, msg, strlen(msg));

    char buf[64];
    int n = serial.readData(buf, sizeof(buf) - 1);

    assert(n > 0);
    buf[n] = '\0';

    assert(std::string(buf).find("hello serial") != std::string::npos);

    close(rpi_fd);
    close(arduino_fd);

    std::cout << "[PASS] test_serial_pty_basic\n";
}

inline void test_serial_partial_read() {
    int rpi_fd, arduino_fd;
    char arduino_name[128];
    assert(openpty(&rpi_fd, &arduino_fd, arduino_name, nullptr, nullptr) == 0);

    Serial serial(arduino_name, B115200);

    write(rpi_fd, "hello ", 6);
    usleep(50000);
    write(rpi_fd, "world\n", 6);

    char buf[64];
    int n = serial.readData(buf, sizeof(buf) - 1);
    assert(n > 0);

    close(rpi_fd);
    close(arduino_fd);
    std::cout << "[PASS] test_serial_partial_read\n";
}

inline void test_serial_timeout() {
    int rpi_fd, arduino_fd;
    char arduino_name[128];
    assert(openpty(&rpi_fd, &arduino_fd, arduino_name, nullptr, nullptr) == 0);

    Serial serial(arduino_name, B115200);

    char buf[64];
    int n = serial.readData(buf, sizeof(buf));
    assert(n <= 0);

    close(rpi_fd);
    close(arduino_fd);
    std::cout << "[PASS] test_serial_timeout\n";
}

inline void test_serial_invalid_device() {
    Serial serial("/dev/ttyNONEXISTENT", B115200);

    assert(serial.isOpen() == false);

    char buf[10];
    int n = serial.readData(buf, sizeof(buf));
    assert(n == -1);

    std::cout << "[PASS] test_serial_invalid_device\n";
}

inline void test_serial() {
    test_serial_pty_basic();
    test_serial_partial_read();
    test_serial_timeout();
    test_serial_invalid_device();
}

#else

// macOS / other OS fallback
inline void test_serial() {
    std::cout << "[SKIP] Serial PTY tests only run on Linux\n";
}

#endif

#endif
