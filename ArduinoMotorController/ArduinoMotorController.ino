#include <Wire.h>
#include "Adafruit_MotorShield.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(2);
void setup() {
  AFMS.begin();  // create with the default frequency 1.6KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(150);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);
}
void loop() {
  myMotor->run(FORWARD);
  myMotor->setSpeed(150); 
  delay(3000);
  myMotor->run(RELEASE);
 
 // myMotor->run(BACKWARD);
  //myMotor->setSpeed(180); 
  //delay(2000);
  //myMotor->run(RELEASE);
  //delay(1000);
}