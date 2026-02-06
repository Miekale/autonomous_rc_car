#include "PurePursuitControls.hpp"


PurePursuitControls::PurePursuitControls(double K_curve, double K_velocity, double lookAheadDist, double maxLinearVel) : 
    K_curve(K_curve), K_velocity(K_velocity), lookAheadDist(lookAheadDist), maxLinearVel(maxLinearVel) {}
    
std::pair<double, double> PurePursuitControls::getControl(Position currPos, Position target, const std::vector<Position>& path) {
    /*
        As of right now, just does pretty traditional pure pursuit 

        Things we might want to include
        - look ahead scaling based on velocity (L + kv)
        - think about miekales fov thing, maybe we cut out any points that are almost behind us

        Things we need
        - goal stopping? Does that live outside of pp?
    */
    // transform target point to locale coords (robot faces down the x)
    double dx = target.x - currPos.x;
    double dy = target.y - currPos.y;
    
    Position localeTarget = rotatePosition(dx, dy, -currPos.theta);

    double steeringAngle = K_curve * 2.0 * localeTarget.y / (lookAheadDist * lookAheadDist);
    // big steven gong said that raceline optimization for velocity could be hype
    double velocity = maxLinearVel / (1 + K_velocity * abs(steeringAngle)); // tune velocity based on curvature
    double angularVel = (2.0 * velocity * localeTarget.y) / (lookAheadDist * lookAheadDist);

    return {velocity, angularVel};
}

Position PurePursuitControls::rotatePosition(double dx, double dy, double angle) {
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
