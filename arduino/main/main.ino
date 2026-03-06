// MAIN LOOP FOR BOTH MOTOR AND CLAW CONTROL
#include <Servo.h>
#include "RobotController.hpp"
#include "PacketHandler.hpp"

// Setup servos and motors ports
Servo clawServo;
Servo esc_left;
Servo esc_right;

RobotController* robot;
PacketHandler* handler;

void setup() {
  Serial.begin(38400);
  robot = new RobotController(clawServo, 9, esc_left, 8, esc_right, 10);
  handler = new PacketHandler(Serial, *robot);
  robot->init_motors();
}

void loop() {
  // robot->execute_v_w_command(10, 0);
  handler->update();
}