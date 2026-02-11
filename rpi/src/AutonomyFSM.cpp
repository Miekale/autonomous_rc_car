#include "AutonomyFSM.hpp"
#include "Constants.hpp"
#include "CustomUtils.hpp"

AutonomyFSM::AutonomyFSM(PurePursuit* pure_pursuit, Perception* perception) {
    _pure_pursuit = pure_pursuit;
    _perception = perception;
    _state = INIT;
}

/**
Steps the FSM.

Can call pure_pursuit or claw_alignment depending on state.
*/
void AutonomyFSM::step() {
    switch (_state) {
        case INIT:
            break;
        case LF_PRE_RESCUE:
            do_lf_pre_rescue();
    }

    _transition_state();
}

void AutonomyFSM::_transition_state() {
    switch (_state) {
        case INIT:
            break;
        
        case LF_PRE_RESCUE:
            // Only exit this state once we see
            // bounding boxes within BULLSEYE_DISTANCE_STOP_LF
            if (get_magnitude(_closest_bullseye) < BULLSEYE_DISTANCE_STOP_LF) {
                _state = RESCUING;
            }

            break;
        
        case RESCUING:
            break;
        
        case LF_POST_RESCUE:
            break;
        
        case DROPPING:
            break;
    }
}


void AutonomyFSM::do_lf_pre_rescue() {
    // Query perception for points list
    std::vector<Position> lf_points = _perception->get_latest_line_follow_points();

    // Run PPS controller
    Position target_point = _pure_pursuit->findLookaheadPoint(Position({0,0,0}), lf_points);
    std::pair<double, double> command = _pure_pursuit->getControl(Position({0,0,0}), target_point, lf_points);

    // TODO: send command to Arduino via interfacing library

    // Query perception for bullseye, update if exists
    auto bullseye = _perception->get_latest_bullsey_point();
    if (bullseye.has_value()) {
        Position bullseye_loc = bullseye.value();
    }

}