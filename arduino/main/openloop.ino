// Open loop motor control example
#include <SimpleFOC.h>


// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(7, 0.09, 1000);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver = BLDCDriver3PWM(11, 10, 9, 8);

// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);


//target variable
float target_velocity = 10.0;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void doLimit(char* cmd) { command.scalar(&motor.voltage_limit, cmd); }

void _setup() {

  // use monitoring with serial 
  Serial.begin(115200);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // limit the maximal dc voltage the driver can set
  // as a protection measure for the low-resistance motors
  // this value is fixed on startup
  driver.voltage_limit = 6;
  
  if(!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }
  // link the motor and the driver
  motor.linkDriver(&driver);
  //motor.velocity_limit = 1200.0;
  // limiting motor movements
  // limit the voltage to be set to the motor
  // start very low for high resistance motors
  // current = voltage / resistance, so try to be well under 1Amp
  motor.voltage_limit = 1.0;   // [V]
  //motor.current_limit = 4.0;
  // open loop control config
  motor.controller = MotionControlType::velocity_openloop;

  // init motor hardware
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");
  
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void _loop() {
  //target_velocity = 20.0;
  motor.move(target_velocity);

  if (target_velocity < 300.0) {
    target_velocity += 0.01;
  }

  // open loop velocity movement
  // using motor.voltage_limit and motor.velocity_limit
  // to turn the motor "backwards", just set a negative target_velocity
  //Serial.println("motor");
  // Serial.println(motor.voltage.d);
  // Serial.println(motor.voltage.q);
  Serial.println(motor.current.q);

  // user communication
  command.run();
}
