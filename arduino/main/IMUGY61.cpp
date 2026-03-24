#include "IMUGY61.hpp"

IMUGY61::IMUGY61(int x_pin, int y_pin, int z_pin) {
    _x_pin = x_pin;
    _y_pin = y_pin;
    _z_pin = z_pin;
}

float IMUGY61::getX() const {
    return X_SCALE * (analogRead(_x_pin) - X_ZERO) / 100.0f;
}

float IMUGY61::getY() const {
    return Y_SCALE * (analogRead(_y_pin) - Y_ZERO) / 100.0f;
}

float IMUGY61::getZ() const {
    return Z_SCALE * (analogRead(_z_pin) - Z_ZERO) / 100.0f;
}