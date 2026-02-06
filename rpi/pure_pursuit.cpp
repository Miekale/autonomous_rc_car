
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>


/*
coulters original pure pursuit paper
https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
big gong
https://stevengong.co/notes/Pure-Pursuit
big thomas
https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PurePursuit.html
*/

struct Position {
    double x, y, theta;
};

class PurePursuit {
private:
    double lookaheadDist;
    double maxLinearVel;
    Position lastPosition{};
    bool hasLastPosition = false;
    double lookAheadTol;
    double K_velocity;
    double K_curve;

public:
    PurePursuit(double lookAheadDist, double maxLinearVel, double lookAheadTol, double K_velocity, double K_curve ) : 
        lookaheadDist(lookAheadDist), maxLinearVel(maxLinearVel), lookAheadTol(lookAheadTol), K_velocity(K_velocity),
        K_curve(K_curve) {}

    std::pair<double, double> getControl(Position currPos, const std::vector<Position>& path) {
        /*
            As of right now, just does pretty traditional pure pursuit 

            Things we might want to include
            - look ahead scaling based on velocity (L + kv)
            - think about miekales fov thing, maybe we cut out any points that are almost behind us

            Things we need
            - goal stopping? Does that live outside of pp?
        */
        Position target = findLookaheadPoint(currPos, path);
        // transform target point to locale coords (robot faces down the x)
        double dx = target.x - currPos.x;
        double dy = target.y - currPos.y;
        
        Position localeTarget = rotatePosition(dx, dy, -currPos.theta);

        double steeringAngle = K_curve * 2.0 * localeTarget.y / (lookaheadDist * lookaheadDist);
        // big steven gong said that raceline optimization for velocity could be hype
        double velocity = maxLinearVel / (1 + K_velocity * abs(steeringAngle)); // tune velocity based on curvature
        double angularVel = (2.0 * velocity * localeTarget.y) / (lookaheadDist * lookaheadDist);

        return {velocity, angularVel};
    }

private:
    Position rotatePosition(double dx, double dy, double angle) {
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

    Position findLookaheadPoint(Position currPos, const std::vector<Position>& path) {
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
            return path.back();
        }

        lastPosition = path[bestIndex];
        hasLastPosition = true;
        return path[bestIndex];
    }
};