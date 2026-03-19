#include <Wire.h>
#include "QGPMaker_MotorShield.h"
#include "QGPMaker_Encoder.h"

QGPMaker_MotorShield AFMS = QGPMaker_MotorShield();
QGPMaker_DCMotor *motors[4];
QGPMaker_Encoder *encoders[4];

// --- CONFIGURATIE ---
const int PPR = 12;
const int gearratio = 90;
const int CPR = (PPR * 4) * gearratio;
const unsigned long sampleTime = 100; // Interval in ms (0.1 seconde)

// CORRECTIE: Richting aangepast omdat motorpolariteit is omgewisseld
int encoderDirection[4] = {-1, -1, 1, 1}; 

// PID Instellingen
double targetRPM = 60.0; 
double Kp = 2.5;         
double Ki = 2.5;         

double integralError[4] = {0, 0, 0, 0};
unsigned long lastMillis = 0;

void setup() {
  AFMS.begin(50);
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    motors[i] = AFMS.getMotor(i + 1);
    encoders[i] = new QGPMaker_Encoder(i + 1);
    motors[i]->run(FORWARD); // Controleer of dit nog de juiste kant op is!
    motors[i]->setSpeed(0);
  }
  
  Serial.println("Systeem gestart. Encoderrichting gecorrigeerd.");
  lastMillis = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastMillis >= sampleTime) {
    double dt = (double)(currentMillis - lastMillis) / 1000.0;
    lastMillis = currentMillis;

    Serial.print("Target:");
    Serial.print(targetRPM);
    Serial.print(" ");

    for (int i = 0; i < 4; i++) {
      // 1. Lees de encoder uit en reset
      long pos = encoders[i]->read() * encoderDirection[i]; 
      encoders[i]->write(0);
      
      // 2. Bereken actuele RPM
      double currentRPM = ((double)pos / (double)CPR) * (60.0 / dt);

      // 3. PI Regeling
      double error = targetRPM - currentRPM;
      
      // Anti-windup & Integral reset bij stilstand
      if (targetRPM == 0) {
        integralError[i] = 0;
      } else {
        integralError[i] += error * dt;
        integralError[i] = constrain(integralError[i], -100, 100); 
      }

      // Bereken output
      double output = (Kp * error) + (Ki * integralError[i]);
      
      // Begrens naar 0-255 PWM
      int finalPWM = constrain((int)output, 0, 255);

      // 4. Stuur motor aan
      motors[i]->setSpeed(finalPWM);

      // 5. Plotter output
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(":");
      Serial.print(currentRPM);
      
      if (i < 3) Serial.print(" ");
    }
    Serial.println();
  }
}