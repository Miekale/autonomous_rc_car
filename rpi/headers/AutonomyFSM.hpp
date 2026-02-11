#ifndef AUTONOMY_FSM_HPP
#define AUTONOMY_FSM_HPP

#include "PurePursuit.hpp"
#include "Perception.hpp"
#include "Constants.hpp"
#include "CustomUtils.hpp"
#include "RescueController.hpp"

enum autonomy_state {
    INIT,
    LF_PRE_RESCUE,
    RESCUING,
    LF_POST_RESCUE,
    DROPPING
};

class AutonomyFSM {
private:
    autonomy_state _state = INIT; 
    PurePursuit* _pure_pursuit = nullptr;
    Perception* _perception = nullptr;
    RescueController* _rescue_controller = nullptr;

    void do_init();
    void do_lf_pre_rescue();
    void do_rescuing();
    void do_lf_post_rescue();
    void do_dropping();

    void _transition_state();

    Position _closest_bullseye;
    Position _goal;
    bool _rescued_lego_person;
    bool _dropped_lego_person;

public:
    // Constructor
    // Takes perception and controller singletons
    AutonomyFSM(PurePursuit *pure_pursuit, Perception *perception, RescueController *rescue_controller);

    // Main interface function: step
    void step();
};

#endif