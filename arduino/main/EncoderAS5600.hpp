#ifndef ENCODER_AS5600_H
#define ENCODER_AS5600_H

#include <SoftWire.h>

class EncoderAS5600 {
private:
    SoftWire wire;
    uint8_t sdaPin;
    uint8_t sclPin;

    uint8_t txBuffer[32];
    uint8_t rxBuffer[32];

    uint16_t prevRaw = 0;
    long totalTicks = 0;

    float velocity = 0.0f;

    static const uint8_t ADDR = 0x36;
    static const uint8_t ANGLE_REG = 0x0C;

    uint16_t readRaw();

public:
    EncoderAS5600(uint8_t sda, uint8_t scl);


    void update(float dt);
    float getVelocity() const;
    void init(uint32_t clock = 10000);
    void update();

    float getAngle() const;
    long  getTicks() const;

    void reset();
};

#endif
