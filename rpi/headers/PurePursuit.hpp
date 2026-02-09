#ifndef PURE_PURSUIT_HPP
#define PURE_PURSUIT_HPP

#include <vector>
#include <cmath>
#include <limits>
#include "Constants.hpp"

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
    double lookaheadDist;
    Position lastPosition{};
    bool hasLastPosition = false;
    double lookAheadTol;

public:
    PurePursuit(double lookAheadDist, double lookAheadTol);

    Position findLookaheadPoint(Position currPos, const std::vector<Position>& path);
};

#endif