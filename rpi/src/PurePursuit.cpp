#include "PurePursuit.hpp"
#include <algorithm>

PurePursuit::PurePursuit(double lookAheadDist, double lookAheadTol) : 
    lookaheadDist(lookAheadDist), lookAheadTol(lookAheadTol) {}

Position PurePursuit::findLookaheadPoint(Position currPos, const std::vector<Position>& path) {
    int bestIndex = 0;
    double minAngleError = std::numeric_limits<double>::max();
    bool foundInRing = false;

    // Loop through all points
    for (int i = 0; i < path.size(); ++i) {
        double dx = path[i].x - currPos.x;
        double dy = path[i].y - currPos.y;
        double dst = std::sqrt(dx * dx + dy * dy);

        // Check if the position is within a tolerance of look ahead
        if (std::abs(dst - lookaheadDist) <= lookAheadTol) {
            
            // find relative angle (in global coords) and normalize
            double angleToPoint = std::atan2(dy, dx);
            double alpha = angleToPoint - currPos.theta;
            alpha = std::atan2(std::sin(alpha), std::cos(alpha));

            // keep running track of position closest to heading 
            double absAlpha = std::abs(alpha);
            if (absAlpha < minAngleError) {
                minAngleError = absAlpha;
                bestIndex = i;
                foundInRing = true;
            }
        }
    }

    if (!foundInRing) { // fallback, just returns the last position for now. 
        if (hasLastPosition) return lastPosition;
        return currPos;
    }

    lastPosition = path[bestIndex];
    hasLastPosition = true;
    return path[bestIndex];
}