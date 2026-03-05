#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include <vector>
#include <cmath>
#include <limits>
#include "Constants.hpp"
#include <utility>

/*
coulters original pure pursuit paper
https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
big gong
https://stevengong.co/notes/Pure-Pursuit
big thomas
https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PurePursuit.html
*/

class PurePursuit {
private:
    Position lastPosition{};
    bool hasLastPosition = false;
    double lookAheadTol;
    double K_velocity;
    double K_curve;
    double lookAheadDist;
    double maxLinearVel;

    Position rotatePosition(double dx, double dy, double angle);

public:
  PurePursuit(double lookAheadDist, double lookAheadTol,
            double K_curve, double K_velocity,
            double maxLinearVel); 
    std::pair<double, double> getControl(Position currPos, Position target, const std::vector<Position>& path);
    Position findLookaheadPoint(Position currPos, const std::vector<Position>& path);

};

#endif