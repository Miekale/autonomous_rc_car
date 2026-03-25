#ifndef IMUGY61_H
#define IMUGY61_H
#include "Arduino.h"
class IMUGY61 {
private:
    int   _x_pin, _y_pin, _z_pin;

    // Per-axis calibration (tune these to your unit)
    static constexpr float X_ZERO  = 344.0f; 
    static constexpr float Y_ZERO  = 345.0f;  
    static constexpr float Z_ZERO  = 334.0f;  

    static constexpr float SCALE = 93.0f;


public:
    IMUGY61(int x_pin, int y_pin, int z_pin);
    float getX() const;
    float getY() const;
    float getZ() const;
};

#endif
