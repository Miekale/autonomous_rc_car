// MAIN LOOP FOR BOTH MOTOR AND CLAW CONTROL
#include <Servo.h>
#include "RobotController.hpp"
#include "PacketHandler.hpp"
#include "IMUGY61.hpp"
#include <Arduino.h>
#include "unistd.h"

// Pin definition
// imu
const int IMU_X_PIN = A0;
const int IMU_Y_PIN = A1;
const int IMU_Z_PIN = A2;

// encoders
const int ENCODER_L_SDA = A8;
const int ENCODER_L_SCL = A9;
const int ENCODER_R_SDA = A10;
const int ENCODER_R_SCL = A11;

// motor
const int MOTOR_L_EN_PIN = 7;
const int MOTOR_R_EN_PIN = 8;

const int MOTOR_L_HIGH_PIN = 9;
const int MOTOR_L_LOW_PIN = 10;
const int MOTOR_R_HIGH_PIN = 11;
const int MOTOR_R_LOW_PIN = 12;
const float LEFT_COMPENSATION = 0.9;

// Declare servos and motors ports
Servo claw_servo;
int claw_servo_pin = 6; 

RobotController* robot;
PacketHandler* handler;
Servo serv;
IMUGY61 imu(IMU_X_PIN, IMU_Y_PIN, IMU_Z_PIN);
EncoderAS5600 leftEncoder(ENCODER_L_SDA, ENCODER_L_SCL);
EncoderAS5600 rightEncoder(ENCODER_R_SDA, ENCODER_R_SCL);

void setup() {
  Serial.begin(38400);
  robot = new RobotController(leftEncoder, rightEncoder, imu, claw_servo, 
        claw_servo_pin, MOTOR_L_HIGH_PIN, MOTOR_L_LOW_PIN, 
        MOTOR_R_HIGH_PIN, MOTOR_R_LOW_PIN);  handler = new PacketHandler(Serial, *robot);
  serv.attach(6);
  //handler->rpiHandshake();
  Serial.println("Handshake done");
  serv.write(60);
}

void loop() {
//  robot->set_m_l_speed(0.3);
//  robot->set_m_r_speed(0.3 * LEFT_COMPENSATION);
//  
//  delay(1000);
//
//  robot->set_m_l_speed(0);
//  robot->set_m_r_speed(0);
//
//  delay(1000);
//
//  robot->set_m_l_speed(-0.3);
//  robot->set_m_r_speed(-0.3 * LEFT_COMPENSATION);
//
//  delay(1000);
//
//  robot->set_m_l_speed(0);
//  robot->set_m_r_speed(0);
//
//  delay(1000);
    handler->update();
} 
