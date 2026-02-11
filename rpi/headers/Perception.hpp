#ifndef PERCEPTION_HPP
#define PERCEPTION_HPP

#include <vector>
#include <optional>

#include "Constants.hpp"

class Perception {
private:
public:
    // Interface to FSM & controllers
    // TODO: miekale you can change these names but make sure to re-name the usages
    // inside FSM as well
    std::vector<Position> get_latest_line_follow_points();
    std::optional<Position> get_latest_bullsey_point();
};

#endif
