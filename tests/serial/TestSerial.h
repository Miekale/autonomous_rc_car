#ifndef TEST_SERIAL_H
#define TEST_SERIAL_H

#include <pty.h>
#include <cstring>
#include <cassert>
#include <iostream>

#include "Serial.hpp"

void test_serial_pty_basic() {
    int rpi_fd, arduino_fd;
    char arduino_name[128];

    // Create pseudo-terminal pair
    int rc = openpty(&rpi_fd, &arduino_fd, arduino_name, nullptr, nullptr);
    assert(rc == 0);

    // Construct serial object using fake device
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
    /*
    For when the arduino is slow and we have drip feed
    */
    int rpi_fd, arduino_fd;
    char arduino_name[128];
    assert(openpty(&rpi_fd, &arduino_fd, arduino_name, nullptr, nullptr) == 0);

    Serial serial(arduino_name, B115200);

    // Send half the message, wait, then send the rest
    write(rpi_fd, "hello ", 6);
    usleep(50000); // 50ms delay
    write(rpi_fd, "world\n", 6);

    char buf[64];
    // We expect to read at least something
    int n = serial.readData(buf, sizeof(buf) - 1);
    assert(n > 0);
    
    close(rpi_fd);
    close(arduino_fd);
    std::cout << "[PASS] test_serial_partial_read\n";
}

inline void test_serial_timeout() {
    /*Tests timeout if the arduino crashes */
    int rpi_fd, arduino_fd;
    char arduino_name[128];
    assert(openpty(&rpi_fd, &arduino_fd, arduino_name, nullptr, nullptr) == 0);

    Serial serial(arduino_name, B115200);

    char buf[64];
    // Arduino sends NOTHING. 
    // Because VMIN=1 and VTIME=1, it should return after ~100ms
    int n = serial.readData(buf, sizeof(buf));

    // If read() returns 0, it means it timed out without reading VMIN bytes
    assert(n <= 0); 

    close(rpi_fd);
    close(arduino_fd);
    std::cout << "[PASS] test_serial_timeout\n";
}

inline void test_serial_invalid_device() {
    /*
    Asserts connection to non existent device doesn't work
    */
    Serial serial("/dev/ttyNONEXISTENT", B115200);

    assert(serial.isOpen() == false);

    char buf[10];
    int n = serial.readData(buf, sizeof(buf));
    assert(n == -1); // Should return error code since fd is -1

    std::cout << "[PASS] test_serial_invalid_device\n";
}

void test_serial() {
    test_serial_pty_basic();
    test_serial_partial_read();
    test_serial_timeout();
    test_serial_invalid_device();
}

#endif
