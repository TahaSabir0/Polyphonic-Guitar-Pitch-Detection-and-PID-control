/*********************************************************************
 * Purpose: Basic motor functionality test.
 * Board: Arduino Uno R4 / Mega 2560
 * Description: A simple sketch to test motor connections and basic
 * movement without any sensor feedback.
 *********************************************************************/

 #include <Arduino.h>

 // Motor 1 pins
const int motor1pin1 = 2;
const int motor1pin2 = 3;


// Enable pins for PWM speed control
const int enableMotor1 = 9;


void setup() {
  // Set motor control pins as OUTPUT
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);


  // Set enable pins as OUTPUT
  pinMode(enableMotor1, OUTPUT);
}


void loop() {
  // Set motor speeds using PWM
  analogWrite(enableMotor1, 100000); // Speed for motor 1


  // Run motors forward
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);


  delay(2000);


  // Reverse motors
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);


  delay(2000);
}



