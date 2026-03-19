// MAIN LOOP FOR BOTH MOTOR AND CLAW CONTROL
#include <Servo.h>
#include "RobotController.hpp"
#include "PacketHandler.hpp"

// Pin definition
const int MOTOR_L_HIGH_PIN = 9;
const int MOTOR_L_LOW_PIN = 10;
const int MOTOR_R_HIGH_PIN = 11;
const int MOTOR_R_LOW_PIN = 12;

// Declare servos and motors ports
Servo claw_servo;
int claw_servo_pin = 7; // TODO CHECK THIS

RobotController* robot;
PacketHandler* handler;

void setup() {
  Serial.begin(38400);
  robot = new RobotController(claw_servo, claw_servo_pin, MOTOR_L_HIGH_PIN, MOTOR_L_LOW_PIN, MOTOR_R_HIGH_PIN, MOTOR_R_LOW_PIN);
  handler = new PacketHandler(Serial, *robot);
  robot->init_motors();
}

void loop() {
  // robot->execute_v_w_command(10, 0);
  
  robot->set_m_l_speed(0.5);
  robot->set_m_r_speed(0.5);
  
  delay(1000);

  robot->set_m_l_speed(0);
  robot->set_m_r_speed(0);

  delay(1000);

  robot->set_m_l_speed(-0.5);
  robot->set_m_r_speed(-0.5);

  delay(1000);

  robot->set_m_l_speed(0);
  robot->set_m_r_speed(0);

  delay(1000);
}
