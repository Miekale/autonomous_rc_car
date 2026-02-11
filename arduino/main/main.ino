// Include the Arduino core library
#include <Arduino.h>

// Define the analog pins connected to the accelerometer
const int xPin = A0;
const int yPin = A1;
const int zPin = A2;

void setup() {
  // Initialize serial communication at 9600 baud rate
  Serial.begin(9600);
}

void loop() {
  // Read the raw values from the accelerometer
  int xRaw = analogRead(xPin);
  int yRaw = analogRead(yPin);
  int zRaw = analogRead(zPin);

  // Convert the raw values to 'g' values
  float xG = 1.5 * (xRaw - 335.0) / 100.0; // Replace 338.0 with your calibrated zero-g value
  float yG = 1.475 * (yRaw - 334.0) / 100.0; // Replace 338.0 with your calibrated zero-g value
  float zG = 1.475 * (zRaw - 344.0) / 100.0; // Replace 338.0 with your calibrated zero-g value

  // Print the acceleration 'g' values to the Serial Monitor
  Serial.print("X: ");
  Serial.print(xG);
  Serial.print("g, Y: ");
  Serial.print(yG);
  Serial.print("g, Z: ");
  Serial.print(zG);
  Serial.println("g");

  // Delay for a bit to avoid spamming the Serial Monitor
  delay(100);
}