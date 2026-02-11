#ifndef AUTONOMY_FSM_HPP
#define AUTONOMY_FSM_HPP

#include "PurePursuit.hpp"
#include "Perception.hpp"
#include "Constants.hpp"
#include "CustomUtils.hpp"

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

    void do_init();
    void do_lf_pre_rescue();
    void do_rescuing();
    void do_lf_post_rescue();
    void do_dropping();

    void _transition_state();

    Position _closest_bullseye;

public:
    // Constructor
    // Takes perception and controller singletons
    AutonomyFSM(PurePursuit *pure_pursuit, Perception *perception);

    // Main interface function: step
    void step();
};

#endif