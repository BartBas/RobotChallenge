#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialiseer de PWM driver op het standaard adres 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x60);

// Definieer de kanalen op de PCA9685 chip (gebaseerd op jouw schema)
// Motor 1 (U3)
const int M1_IN1 = 8;
const int M1_IN2 = 9;

// Motor 3 (U4 - onder U3)
const int M3_IN1 = 10;
const int M3_IN2 = 11;

// Motor 2 (U5 - rechts van U3)
const int M2_IN1 = 12;
const int M2_IN2 = 13;

// Motor 4 (U6 - onder U5)
const int M4_IN1 = 14;
const int M4_IN2 = 15;

void setup() {
  Serial.begin(9600);
  Serial.println("Motor Test Start");

  pwm.begin();
  // Zet frequentie op 1000Hz (goed voor DC motoren met DRV8870)
  // De standaard 50Hz is vaak te laag en veroorzaakt gezoem.
  pwm.setPWMFreq(1000); 
}

void loop() {
  // --- VOORBEELD ROUTINE ---
  
  // 1. Alle motoren VOORUIT op halve kracht
  Serial.println("Alle motoren vooruit (50%)");
  setMotor(1, 4095); //rechts achter
  setMotor(2, -4095); //links achter
  setMotor(3, -4095); //rechts voor
  setMotor(4, 4095); //links voor
  delay(2000);

  // 2. STOP
  //Serial.println("Stop");
  stopAll();
  delay(1000);

  // 3. Alle motoren ACHTERUIT op volle kracht
 // Serial.println("Alle motoren achteruit (100%)");
  //setMotor(1, -2048); // Negatief getal = achteruit
  //setMotor(2, -2048);
  //setMotor(3, -2048);
  //setMotor(4, -2048);
  //delay(2000);

  // 4. STOP
 // stopAll();
  //delay(1000);
  
  // 5. Draaien (Links motoren vooruit, Rechts achteruit)
 // Serial.println("Draaien...");
 // setMotor(1, 3000); // Links voor
 // setMotor(3, 3000); // Links achter
 // setMotor(2, -3000); // Rechts voor
  //setMotor(4, -3000); // Rechts achter
  //delay(2000);
  
  //stopAll();
  //delay(2000);
}

// --- HULP FUNCTIES ---

/**
 * Stuur een motor aan.
 * @param motorID: 1, 2, 3 of 4
 * @param speed: Snelheid tussen -4095 (vol achteruit) en 4095 (vol vooruit). 0 is stop.
 */
void setMotor(int motorID, int speed) {
  int pinA, pinB;

  // Koppel ID aan pinnen
  switch (motorID) {
    case 1: pinA = M1_IN1; pinB = M1_IN2; break;
    case 2: pinA = M2_IN1; pinB = M2_IN2; break;
    case 3: pinA = M3_IN1; pinB = M3_IN2; break;
    case 4: pinA = M4_IN1; pinB = M4_IN2; break;
    default: return;
  }

  if (speed > 0) {
    // Vooruit
    if (speed > 4095) speed = 4095;
    pwm.setPWM(pinA, 0, speed); // Pin A krijgt PWM signaal
    pwm.setPWM(pinB, 0, 0);     // Pin B is volledig LAAG
  } else if (speed < 0) {
    // Achteruit
    speed = -speed; // Maak positief voor PWM waarde
    if (speed > 4095) speed = 4095;
    pwm.setPWM(pinA, 0, 0);     // Pin A is volledig LAAG
    pwm.setPWM(pinB, 0, speed); // Pin B krijgt PWM signaal
  } else {
    // Stop (coast)
    pwm.setPWM(pinA, 0, 0);
    pwm.setPWM(pinB, 0, 0);
  }
}

void stopAll() {
  setMotor(1, 0);
  setMotor(2, 0);
  setMotor(3, 0);
  setMotor(4, 0);
}