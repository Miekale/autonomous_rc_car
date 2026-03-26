#include "EncoderAS5600.hpp"

EncoderAS5600::EncoderAS5600(uint8_t sda, uint8_t scl)
    : wire(sda, scl), sdaPin(sda), sclPin(scl) {}

void EncoderAS5600::init(bool invert = false, uint32_t clock = 10000) {
    pinMode(sdaPin, INPUT_PULLUP);
    pinMode(sclPin, INPUT_PULLUP);

    _invert = invert;

    wire.setTxBuffer(txBuffer, 32);
    wire.setRxBuffer(rxBuffer, 32);
    wire.begin();
    wire.setClock(clock);

    prevRaw = readRaw();
}

uint16_t EncoderAS5600::readRaw() {
    /*
    AS5600 is 12 bit so returns a 12 bit int angle. 
    we begin transmission at address (0x36 for this device) and request the angle
    */
    wire.beginTransmission(ADDR);
    wire.write(ANGLE_REG);

    if (wire.endTransmission(false) != 0)
        return 0xFFFF;

    // request two bytes
    wire.requestFrom(ADDR, (uint8_t)2);

    // confirm we get both bytes and read
    if (wire.available() >= 2) {
        uint8_t high = wire.read();
        uint8_t low  = wire.read();
        // only the lower 4 bytes of high are valid, so just mask to receive those
        return ((high & 0x0F) << 8) | low;
    }

    // just an invalid value above 4095
    return 0xFFFF;
}

float EncoderAS5600::getAngle() const {
    return totalTicks * (2.0f * PI / 4096.0f);
}

long EncoderAS5600::getTicks() const {
    return totalTicks;
}

void EncoderAS5600::reset() {
    totalTicks = 0;
}

// In update(), track velocity internally
void EncoderAS5600::update(float dt) {
    
    uint16_t currentRaw = readRaw();
    
    if (currentRaw > 4095) return;
    

    int16_t delta = currentRaw - prevRaw;

    if (delta > 2048)       delta -= 4096;
    else if (delta < -2048) delta += 4096;

    if (_invert) delta = -delta;
    totalTicks += delta;
    prevRaw = currentRaw;
    if (dt > 0.001f) {
        velocity = (delta * (2.0f * PI / 4096.0f)) / dt;  // rad/s
    }
    
}

float EncoderAS5600::getVelocity() const { return velocity; }
