#include "IMUGY61.hpp"

IMUGY61::IMUGY61(int x_pin, int y_pin, int z_pin) {
    _x_pin = x_pin;
    _y_pin = y_pin;
    _z_pin = z_pin;
}

float IMUGY61::getX() const {
    return (analogRead(_x_pin) - X_ZERO) / SCALE;
}

float IMUGY61::getY() const {
    return (analogRead(_y_pin) - Y_ZERO) / SCALE;
}

float IMUGY61::getZ() const {
    return (analogRead(_z_pin) - Z_ZERO) / SCALE;
}