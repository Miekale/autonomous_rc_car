#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include <vector>
#include <cmath>
#include <limits>
#include "Constants.hpp"
#include <utility>
#include <algorithm>
#include <iostream>
#include <cmath>

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
    float lookAheadTol;
    float K_velocity;
    float K_curve;
    float lookAheadDist;
    float maxLinearVel;

    float angleTol = 3.14159f / 4.0f;

    Position rotatePosition(float dx, float dy, float angle);

public:
  PurePursuit(float lookAheadDist, float lookAheadTol,
            float K_curve, float K_velocity,
            float maxLinearVel); 
    std::pair<float, float> getControl(Position currPos, Position target, const std::vector<Position>& path);
    Position findLookaheadPoint(Position currPos, const std::vector<Position>& path);

};

#endif