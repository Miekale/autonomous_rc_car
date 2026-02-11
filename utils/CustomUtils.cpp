#include "CustomUtils.hpp"

double get_magnitude(Position point) {
    return std::pow(point.x, 2) + std::pow(point.y, 2);
}