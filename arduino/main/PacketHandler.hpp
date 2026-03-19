#ifndef PACKET_HANDLER_HPP
#define PACKET_HANDLER_HPP

#include <Arduino.h> 
#include <Stream.h> 
#include "RobotController.hpp" // MUST include this to use RobotController&

class PacketHandler {
private:
    Stream& _serial;
    RobotController& _robot;
    float _handshakeEpoch = 0.0f;
    unsigned long _handshakeMillis = 0;


public:
    PacketHandler(Stream& s, RobotController& r);
    void update();
    bool rpiHandshake();

private:
    bool validateChecksum(uint8_t len, uint8_t* data, uint8_t receivedXor);
    void dispatch(uint8_t* data, uint8_t len);
    bool waitForStartupPing();
};

#endif