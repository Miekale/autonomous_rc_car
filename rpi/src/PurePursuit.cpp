#include "PurePursuit.hpp"
#include <algorithm>

PurePursuit::PurePursuit(double lookAheadDist, double lookAheadTol, double K_curve, 
        double K_velocity, double maxLinearVel) : 
    lookAheadDist(lookAheadDist), lookAheadTol(lookAheadTol), K_curve(K_curve), K_velocity(K_velocity), 
     maxLinearVel(maxLinearVel) {}

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
        if (std::abs(dst - lookAheadDist) <= lookAheadTol) {
            
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

std::pair<double, double> PurePursuit::getControl(Position currPos, Position target, const std::vector<Position>& path) {

    // transform target point to locale coords (robot faces down the x)
    double dx = target.x - currPos.x;
    double dy = target.y - currPos.y;
    
    Position localeTarget = rotatePosition(dx, dy, -currPos.theta);

    double steeringAngle = K_curve * 2.0 * localeTarget.y / (lookAheadDist * lookAheadDist);
    //
    double velocity = maxLinearVel / (1 + K_velocity * abs(steeringAngle)); // tune velocity based on curvature
    double angularVel = (2.0 * velocity * localeTarget.y) / (lookAheadDist * lookAheadDist);

    return {velocity, angularVel};
}

Position PurePursuit::rotatePosition(double dx, double dy, double angle) {
    /*
    Just a 2D rotation matrix on input (dx, dy) vector by the given angle
    cos -sin
    sin cos
    */
    double cosA = std::cos(angle);
    double sinA = std::sin(angle);

    Position rotated{0,0,0};
    rotated.x = dx * cosA - dy * sinA;
    rotated.y = dx * sinA + dy * cosA;
    
    return rotated;
}
