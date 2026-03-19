#include "PacketHandler.hpp"

PacketHandler::PacketHandler(Stream& s, RobotController& r) : 
    _serial(s), _robot(r) 
{
}

void PacketHandler::update() {
    if (_serial.available() < 3) return; 

    if (_serial.peek() != 0xAA) {
        _serial.read(); 
        return;
    }

    _serial.read(); // consume header
    uint8_t len = _serial.read();

    unsigned long start = millis();
    while (_serial.available() < len + 1) {
        if (millis() - start > 30) return;  // just some timeout catching
    }

    uint8_t infobuf[len];
    _serial.readBytes(infobuf, len);
    uint8_t receivedCRC = _serial.read();

    if (validateChecksum(len, infobuf, receivedCRC)) {
        dispatch(infobuf, len);
        _serial.write(0x06); // acknowledgement
    } else {
        _serial.write(0x15); // we failed lol
    }
}

bool PacketHandler::validateChecksum(uint8_t len, uint8_t* data, uint8_t receivedXor) {
    uint8_t xor_ = 0xAA ^ len;
    for (uint8_t i = 0; i < len; i++) {
        xor_ ^= data[i];
    }
    return (xor_ == receivedXor);
}

void PacketHandler::dispatch(uint8_t* data, uint8_t len) {
    uint8_t cmdID = data[0];
    switch (cmdID) {
        case 0x00: { // Pure Pursuit
            if (len != 9) return;
            float v, w;
            memcpy(&v, &data[1], 4);
            memcpy(&w, &data[5], 4);
            
            _robot.execute_v_w_command(v, w);
            
            break;
        }
        case 0x01: 
            _robot.openClaw();  
            break;
        case 0x02: 
            _robot.closeClaw(); 
            break;
        default:
            break;
    }
}

bool PacketHandler::waitForStartupPing() {

    if (_serial.available() < 3)
        return false;

    if (_serial.peek() != 0xAA) {
        _serial.read();  // discard junk
        return false;
    }

    _serial.read();           // consume header
    uint8_t len = _serial.read();

    unsigned long start = millis();
    while (_serial.available() < len + 1) {
        if (millis() - start > 30)
            return false;
    }

    uint8_t infobuf[len];
    _serial.readBytes(infobuf, len);
    uint8_t receivedCRC = _serial.read();

    if (!validateChecksum(len, infobuf, receivedCRC)) {
        _serial.write(0x15); // NACK
        return false;
    }

    // Expect exactly: payload = {0x08}
    if (len == 1 && infobuf[0] == 0x08) {
        _serial.write(0x06); // ACK
        return true;
    }

    _serial.write(0x15);
    return false;
}

bool PacketHandler::rpiHandshake() {
    _serial.write(0x55);   // READY signal

    unsigned long start = millis();

    while (millis() - start < 1000) {
        if (this->waitForStartupPing()) {
            _serial.println("Handshake received");
            return true;
        }
    }

    _serial.println("Handshake timeout");
    return false;
    
}