#include "PurePursuit.hpp"

PurePursuit::PurePursuit(float lookAheadDist, float lookAheadTol, float K_curve, 
        float K_velocity, float maxLinearVel) : 
    lookAheadDist(lookAheadDist), lookAheadTol(lookAheadTol), K_curve(K_curve), K_velocity(K_velocity), 
     maxLinearVel(maxLinearVel) {}

Position PurePursuit::findLookaheadPoint(Position currPos, const std::vector<Position>& path) {
    int bestIndex = 0;
    float minAngleError = std::numeric_limits<float>::max();
    bool foundInRing = false;

    // Loop through all points
    for (int i = 0; i < path.size(); ++i) {
        float dx = path[i].x - currPos.x;
        float dy = path[i].y - currPos.y;
        float dst = std::sqrt(dx * dx + dy * dy);

        // Check if the position is within a tolerance of look ahead
        if (std::abs(dst - lookAheadDist) <= lookAheadTol) {
            
            // find relative angle (in global coords) and normalize
            float angleToPoint = std::atan2(dy , dx);
            float alpha = angleToPoint - currPos.theta;
            alpha = std::atan2(std::sin(alpha), std::cos(alpha));

            // keep running track of position closest to heading 
            float absAlpha = std::abs(alpha);
            if (absAlpha < minAngleError) {
                minAngleError = absAlpha;
                bestIndex = i;
                foundInRing = true;
            }
        }
    }

    if (!foundInRing) { // fallback, just returns the last position for now. 
        std::cout << "findLookaheadPoint: not found in ring" << std::endl;
        if (hasLastPosition) return lastPosition;
        return currPos;
    }

    lastPosition = path[bestIndex];
    hasLastPosition = true;
    return path[bestIndex];
}

std::pair<float, float> PurePursuit::getControl(Position currPos, Position target, const std::vector<Position>& path) {

    // transform target point to locale coords (robot faces down the x)
    float dx = target.x - currPos.x;
    float dy = target.y - currPos.y;

    float turn_error = std::atan2(dy, dx);
    float distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

    if (distance < 10) {
        return {0.0, 0.0};
    }
    
    Position localeTarget = rotatePosition(dx, dy, -currPos.theta);

    float scaled_curvature = K_curve * 2.0 * localeTarget.y / (std::pow(distance, 2));
    std::cout << "getControl: " << "local target: " << localeTarget.x << ", " << localeTarget.y << std::endl;
    std::cout << "getControl: " << "scaled_curvature: " << scaled_curvature << std::endl;

    float velocity = maxLinearVel / (1 + K_velocity * abs(scaled_curvature)); // tune velocity based on curvature
    float turn_rel_speed = WHEEL_TO_WHEEL_DISTANCE * std::sin(turn_error) * velocity / LOOK_AHEAD_DISTANCE;
    float angularVel = 2 * turn_rel_speed / WHEEL_TO_WHEEL_DISTANCE * K_VELOCITY;

    return {velocity, angularVel};
}

Position PurePursuit::rotatePosition(float dx, float dy, float angle) {
    /*
    Just a 2D rotation matrix on input (dx, dy) vector by the given angle
    cos -sin
    sin cos
    */
    float cosA = std::cos(angle);
    float sinA = std::sin(angle);

    Position rotated{0,0,0};
    rotated.x = dx * cosA - dy * sinA;
    rotated.y = dx * sinA + dy * cosA;
    
    return rotated;
}
