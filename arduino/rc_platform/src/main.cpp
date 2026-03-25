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
    Serial.begin(115200);

    robot = new RobotController(leftEncoder, rightEncoder, imu, claw_servo, SERVO_CLAW_PIN, MOTOR_L_HIGH_PIN, MOTOR_L_LOW_PIN, MOTOR_L_EN_PIN, MOTOR_R_HIGH_PIN, MOTOR_R_LOW_PIN, MOTOR_R_EN_PIN); 
    handler = new PacketHandler(Serial, *robot);
    
    leftEncoder.init();
    rightEncoder.init(true);

    controls.init(0.0f, 0.0f);
    last_enc_time = millis();
    start_time = millis();

    Serial.println(F("Starting PID tuning run..."));
}

void loop() {
    uint32_t now = millis();
    dt = (now - last_enc_time) / 1000.0f;
    last_enc_time = now;
    handler->update();
    robot->execute_v_w_command();
}

// uint32_t time = 0;
// float target_left_wheel = 25;  //rad/s
// float target_right_wheel = 25;

// float kp_left = 2.5;
// float ki_left = .5;
// float accum_error_left = 0;

// float kp_right = kp_left;
// float ki_right =2.5;
// float accum_error_right = 0;
// void loop() {
//     uint32_t now = millis();
//     dt = (now - last_enc_time) / 1000.0f;
//     last_enc_time = now;
//     handler->update();

//     leftEncoder.update(dt);
//     rightEncoder.update(dt);

//     Serial.print("dt: ");Serial.println(dt, 4);
//     float left_vel = leftEncoder.getVelocity();
//     Serial.print("left vel:");Serial.println(left_vel);
//     float right_vel = rightEncoder.getVelocity();
//     Serial.print("right vel: ");Serial.println(right_vel);

//     Serial.println("left PID ------");
//     float left_wheel_pwm = controls.pid_calculate(
//         target_left_wheel, 
//         left_vel,
//         kp_left, 
//         ki_left,
//         accum_error_left,
//         dt);

//     Serial.println("right PID ------");
//     float right_wheel_pwm = controls.pid_calculate(
//         target_left_wheel, 
//         right_vel,
//         kp_right, 
//         ki_right,
//         accum_error_right,
//         dt);

//     // FF
//     float left_ff = controls.ff(target_left_wheel, false);
//     float right_ff = controls.ff(target_right_wheel, true);

//     // Clamp
//     Serial.print("left FF:");Serial.println(left_ff);
//     Serial.print("left PI:");Serial.println(left_wheel_pwm);
//     Serial.print("right FF:");Serial.println(right_ff);
//     Serial.print("right PI:");Serial.println(right_wheel_pwm);

//     left_wheel_pwm += left_ff;
//     right_wheel_pwm += right_ff;
    
//     Serial.print("left_wheel_pwm pre clamp: ");
//     Serial.println(left_wheel_pwm);
//     Serial.println("right_wheel_pwm pre clamp: ");
//     Serial.println(right_wheel_pwm);

//     left_wheel_pwm = min(left_wheel_pwm, 255);
//     left_wheel_pwm = max(left_wheel_pwm, 0);
//     right_wheel_pwm = min(left_wheel_pwm, 255);
//     right_wheel_pwm = max(left_wheel_pwm, 0);

//     digitalWrite(MOTOR_L_HIGH_PIN, HIGH);
//     digitalWrite(MOTOR_L_LOW_PIN, LOW);
//     analogWrite(MOTOR_L_EN_PIN, left_wheel_pwm);

//     digitalWrite(MOTOR_R_HIGH_PIN, HIGH);
//     digitalWrite(MOTOR_R_LOW_PIN, LOW);
//     analogWrite(MOTOR_R_EN_PIN, right_wheel_pwm);
// }



// float pid = 65;
// float left_wheel_pwm = pid;
// float right_wheel_pwm = pid;

// void loop() {
//     Serial.println("RUNNING AT: ");
//     Serial.println(left_wheel_pwm);
//     Serial.println(right_wheel_pwm);

//     uint32_t now = millis();
//     dt = (now - last_enc_time) / 1000.0f;
//     last_enc_time = now;

//     leftEncoder.update(dt);
//     rightEncoder.update(dt);

//     float left_vel = leftEncoder.getVelocity();
//     Serial.print("left vel: ");Serial.println(left_vel);
//     float right_vel = leftEncoder.getVelocity();
//     Serial.print("right vel: ");Serial.println(right_vel);

//     digitalWrite(MOTOR_L_HIGH_PIN, HIGH);
//     digitalWrite(MOTOR_L_LOW_PIN, LOW);
//     analogWrite(MOTOR_L_EN_PIN, left_wheel_pwm);

//     digitalWrite(MOTOR_R_HIGH_PIN, HIGH);
//     digitalWrite(MOTOR_R_LOW_PIN, LOW);
//     analogWrite(MOTOR_R_EN_PIN, right_wheel_pwm);
// }
