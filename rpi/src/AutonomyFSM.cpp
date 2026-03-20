#include "AutonomyFSM.hpp"
#include "Constants.hpp"
#include "CustomUtils.hpp"
#include "RescueController.hpp"
#include <cstdlib>

AutonomyFSM::AutonomyFSM(PurePursuit* pure_pursuit, Perception* perception, RescueController* rescue_controller, Serial* serial) {
    _pure_pursuit = pure_pursuit;
    _perception = perception;
    _rescue_controller = rescue_controller;
    _serial = serial;
    _state = INIT;

    _closest_bullseye = Position({1000, 1000, 0});
    _goal = Position({1000, 1000, 0});
    _rescued_lego_person = false;
    _dropped_lego_person = false;
    
}

/**
Steps the FSM.

Can call pure_pursuit or rescue_controller depending on state.
*/
void AutonomyFSM::step() {
    switch (_state) {
        case INIT:
            break;
        case LF_PRE_RESCUE:
            do_lf_pre_rescue();
            break;
        case RESCUING:
            do_rescuing();
            break;
        case LF_POST_RESCUE:
            do_lf_post_rescue();
            break;
        case DROPPING:
            do_dropping();
            break;
    }

    _transition_state();
}

void AutonomyFSM::_transition_state() {
    switch (_state) {
        case INIT:
            std::cout << "Transitioned to: LF_PRE_RESCUE state" << std::endl;
            _state = LF_PRE_RESCUE;
            break;
        
        case LF_PRE_RESCUE:
            // Only exit this state once we see
            // bounding boxes within BULLSEYE_DISTANCE_STOP_LF
            if (get_magnitude(_closest_bullseye) < BULLSEYE_DISTANCE_STOP_LF) {
                std::cout << "Transitioned to: RESCUING state" << std::endl;
                _state = RESCUING;
            }

            break;
        
        case RESCUING:
            // Only exit this state if the rescue controller signals
            // that we're ready to LF again.
            if (_rescued_lego_person) {
            std::cout << "Transitioned to: LF_POST_RESCUE state" << std::endl;
                _state = LF_POST_RESCUE;
            }
            break;
        
        case LF_POST_RESCUE:
            if (get_magnitude(_goal) < GOAL_DISTANCE_STOP_LF) {
                std::cout << "Transitioned to: DROPPING state" << std::endl;
                _state = DROPPING;
            }
            break;
        
        case DROPPING:
            if (_dropped_lego_person) {
                // Succesfully Finished the challenge!
                std::cout << "Transitioned to: DONE state" << std::endl;
                exit(0);  // TODO: do actual cleanup, etc. don't just close
            }
            break;
    }
}


void AutonomyFSM::do_lf_pre_rescue() {
    // Query perception for points list
    double timestamp = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count(); // seconds with fractional part

    _perception->get_latest_bgr_frame();
    std::vector<Position> lf_points = _perception->get_latest_line_follow_points_2d();
    std::cout << "do_lf_pre_rescue: Got " << lf_points.size() << " lf_points" << std::endl;
    if (lf_points.size() > 1){
        std::cout << "do_lf_pre_rescue: lf_points " << lf_points[0].x << ", " << lf_points[0].y << std::endl;
    }

    // Run PPS controller
    // TODO: the heading angle in world coords is always 0
    Position target_point = _pure_pursuit->findLookaheadPoint(Position({0,0,0}), lf_points);
    std::cout << "do_lf_pre_rescue: target_point " << target_point.x << ", " << target_point.y << std::endl;
    std::pair<float, float> command = _pure_pursuit->getControl(Position({0,0,0}), target_point, lf_points);
    std::cout << "do_lf_pre_rescue: Got command " << command.first << ", " << command.second << std::endl;

    std::cout << "do_lf_pre_rescue: " << std::endl;
    _rescue_controller->step_pursuit(*_serial, command, timestamp);

    // Query perception for bullseye, update if exists
    std::optional<Position> bullseye = _perception->get_latest_bullsey_point();
    if (bullseye) {
        _closest_bullseye.x = bullseye->x;
        _closest_bullseye.y = bullseye->y;
        std::cout << "Updated closest bullseye to: " << bullseye.value().x << ", " << bullseye.value().y << std::endl;
        _closest_bullseye.theta = bullseye->theta;
    }
}

void AutonomyFSM::do_rescuing() {
    // Step the Rescue state machine
     double timestamp = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count(); // seconds with fractional part
    _rescued_lego_person = _rescue_controller->step_grab(*_serial, timestamp);
}

void AutonomyFSM::do_lf_post_rescue() {
    // Query perception for points list
    double timestamp = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count(); // seconds with fractional part
    std::vector<Position> lf_points = _perception->get_latest_line_follow_points();

    // Run PPS controller
    Position target_point = _pure_pursuit->findLookaheadPoint(Position({0,0,0}), lf_points);
    std::pair<float, float> command = _pure_pursuit->getControl(Position({0,0,0}), target_point, lf_points);

    // TODO: send command to Arduino via interfacing library
    _rescue_controller->step_pursuit(*_serial, command, timestamp);

    // Query perception for goal/dropoff
    std::optional<Position> end_goal = _perception->get_latest_end_goal_point();
    if (end_goal) {
        _goal = *end_goal;
    }
}

void AutonomyFSM::do_dropping() {
    // Step the Rescue state machine
    double timestamp = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count(); // seconds with fractional part
    _dropped_lego_person = _rescue_controller->step_drop(*_serial, timestamp);
}
