#include <Servo.h>
#include "Controls.hpp"
#include "RobotController.hpp"
#include "PacketHandler.hpp"
#include "EncoderAS5600.hpp"
#include "IMUGY61.hpp"
#include <Arduino.h>

// Pins
const int IMU_X_PIN = A0;
const int IMU_Y_PIN = A1;
const int IMU_Z_PIN = A2;

const int ENCODER_L_SDA = A8;
const int ENCODER_L_SCL = A9;
const int ENCODER_R_SDA = A10;
const int ENCODER_R_SCL = A11;

const int MOTOR_L_EN_PIN  = 7;
const int MOTOR_R_EN_PIN  = 8;
const int MOTOR_L_HIGH_PIN = 9;
const int MOTOR_L_LOW_PIN  = 10;
const int MOTOR_R_HIGH_PIN = 11;
const int MOTOR_R_LOW_PIN  = 12;

const int SERVO_CLAW_PIN = 6; 

IMUGY61 imu(IMU_X_PIN, IMU_Y_PIN, IMU_Z_PIN);
EncoderAS5600 leftEncoder(ENCODER_L_SDA, ENCODER_L_SCL);
EncoderAS5600 rightEncoder(ENCODER_R_SDA, ENCODER_R_SCL);
RobotController* robot;
PacketHandler* handler;
Servo claw_servo;
Controls controls;

uint32_t last_enc_time = 0;
uint32_t start_time = 0;
float dt = 0.0f;

void setup() {
    Serial.begin(460800);

    robot = new RobotController(leftEncoder, rightEncoder, imu, claw_servo, SERVO_CLAW_PIN, MOTOR_L_HIGH_PIN, MOTOR_L_LOW_PIN, MOTOR_L_EN_PIN, MOTOR_R_HIGH_PIN, MOTOR_R_LOW_PIN, MOTOR_R_EN_PIN); 

    leftEncoder.init();
    rightEncoder.init(true);

    controls.init(0.0f, 0.0f);
    last_enc_time = millis();
    start_time = millis();

    Serial.println(F("Starting KF tuning run..."));
}

float get_target_speed(uint32_t t_ms) {
    uint32_t elapsed = t_ms - start_time;
    if (elapsed < 2000)       return 0.0f;
    else if (elapsed < 5000)  return 0.01f;
    else                      return 0.0f;
}
uint32_t time = 0;
int second_segment = 0;
int pwm_speed = 10;
void loop() {
    digitalWrite(MOTOR_L_HIGH_PIN, HIGH);
    digitalWrite(MOTOR_L_LOW_PIN, LOW);
    analogWrite(MOTOR_L_EN_PIN, pwm_speed);

    digitalWrite(MOTOR_R_HIGH_PIN, HIGH);
    digitalWrite(MOTOR_R_LOW_PIN, LOW);
    analogWrite(MOTOR_R_EN_PIN, pwm_speed);

    leftEncoder.update(dt);
    rightEncoder.update(dt);

    Serial.print(F("w_l: ")); Serial.println(leftEncoder.getVelocity());
    Serial.print(F(" w_r: ")); Serial.println(rightEncoder.getVelocity());

    uint32_t now = millis();
    dt = (now - last_enc_time) / 1000.0f;
    // Serial.print("Current speed: ");
    // Serial.println(pwm_speed);
    Serial.print(F(" dt: ")); 
    Serial.print(dt, 4);
    // Serial.print(" actual ms time: ");
    // Serial.println(now - last_enc_time);

    robot->execute_v_w_command(1000, 0);  // 1m/s = 1000mm/s
}