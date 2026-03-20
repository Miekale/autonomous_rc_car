#ifndef TEST_CONTROLS_H
#define TEST_CONTROLS_H

#include <iostream>

#include "Controls.hpp"
#include "LinAlg.hpp"

/*
Tests the Kalman Filter.
*/
void test_kf_loop() {
    Controls controller;

    // Stationary robot with starting non-zero estimate.
    // Estimated state should converge to zero.
    controller.estimated_state = Vec3(1, 0, 1);
    Mat3 P(std::array<float, 9>{1,0, 0,0,1,0,0,0, 1});

    // At 50hz, this is after 2 seconds
    for (int i = 0; i < 100; i++) {
        auto pred_output = controller.correct_step(
            Vec3(0, 0, 0), 
            controller.H_enc, 
            controller.R_enc, 
            controller.estimated_state, P);

        std::cout << std::endl;
        std::cout << "After seeing 0,0 encoders for " << i << " steps" ;
        std::cout << pred_output.first << std::endl;
        controller.estimated_state = pred_output.first;
    }
}

void test_ik() {}

void test_fk() {}

void test_forward_path() {}

#endif