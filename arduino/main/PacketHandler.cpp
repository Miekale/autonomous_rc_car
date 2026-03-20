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

    _serial.read();
    uint8_t len = _serial.read();

    unsigned long start = millis();
    while (_serial.available() < len + 1) {
        if (millis() - start > 30) return;
    }

    uint8_t infobuf[len];
    _serial.readBytes(infobuf, len);
    uint8_t receivedCRC = _serial.read();

    if (validateChecksum(len, infobuf, receivedCRC)) {
        // Extract timestamp from the END of infobuf (last 8 bytes before checksum)
        // layout: [cmdID] [data bytes...] [seconds float] [microseconds float]
        float seconds, microseconds;
        memcpy(&seconds,      &infobuf[len - 8], 4);
        memcpy(&microseconds, &infobuf[len - 4], 4);

        float timeSinceSend = seconds + (microseconds / 1e6f); // already a delta from handshake
        float arduinoElapsed = (millis() - _handshakeMillis) / 1000.0f;
        float latency = arduinoElapsed - timeSinceSend;

        Serial.print("Latency since send: ");
        Serial.print(latency * 1000.0f, 3);
        Serial.println(" ms");

        dispatch(infobuf, len);

        _serial.write(0x06);
    } else {
        _serial.write(0x15);
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
            if (len != 17) return; // data package size - change back to 9 when we remove the timestamp stuff
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

bool PacketHandler::rpiHandshake() {
    _serial.write(0x55); // READY signal

    unsigned long start = millis();

    while (millis() - start < 5000) {
        if (this->waitForStartupPing()) {
            _serial.println("Handshake received");
            return true;
        }
    }

    _serial.println("Handshake timeout");
    return false;
}

bool PacketHandler::waitForStartupPing() {
    if (_serial.available() < 3) return false;

    if (_serial.peek() != 0xAA) {
        _serial.read();
        return false;
    }

    _serial.read();
    uint8_t len = _serial.read();

    unsigned long start = millis();
    while (_serial.available() < len + 1) {
        if (millis() - start > 30) return false;
    }

    uint8_t infobuf[len];
    _serial.readBytes(infobuf, len);
    uint8_t receivedCRC = _serial.read();

    if (!validateChecksum(len, infobuf, receivedCRC)) {
        _serial.write(0x15);
        return false;
    }

    // cmd 0x08 + 2 floats (seconds + microseconds) = 9 bytes
    if (len == 9 && infobuf[0] == 0x08) {
        _handshakeMillis = millis(); // just record Arduino time, epoch not needed anymore
        _serial.write(0x06);
        return true;
    }

    _serial.write(0x15);
    return false;
}
