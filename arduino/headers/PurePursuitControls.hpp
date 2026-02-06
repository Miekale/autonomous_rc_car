#ifndef PURE_PURSUIT_CONTROLS_HPP
#define PURE_PURSUIT_CONTROLS_HPP

#include <vector>
#include <cmath>
#include <utility>
#include "Constants.hpp"

class PurePursuitControls {
private:
    double K_velocity;
    double K_curve;
    double lookAheadDist;
    double maxLinearVel;

    Position rotatePosition(double dx, double dy, double angle);

public:
    PurePursuitControls(double K_curve, double K_velocity, double lookAheadDist, double maxLinearVel);
    
    std::pair<double, double> getControl(Position currPos, Position target, const std::vector<Position>& path);
};
#endif