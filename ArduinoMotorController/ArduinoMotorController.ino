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
const unsigned long sampleTime = 100; // Interval in milliseconden (0.1 seconde)

// Correctie voor draairichting encoders per motor
int encoderDirection[4] = {-1, -1, 1, 1}; 

// PID Instellingen (Pas deze waarden aan tijdens het testen)
double targetRPM = 60.0; 
double Kp = 2.5;         // Proportioneel: reageert direct op de fout
double Ki = 2.5;         // Integraal: elimineert de blijvende afwijking

double integralError[4] = {0, 0, 0, 0};
unsigned long lastMillis = 0;

void setup() {
  AFMS.begin(50);
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    motors[i] = AFMS.getMotor(i + 1);
    encoders[i] = new QGPMaker_Encoder(i + 1);
    motors[i]->run(FORWARD);
    motors[i]->setSpeed(0);
  }
  
  Serial.println("Systeem gestart. Modus: Standaard PI-regeling.");
  lastMillis = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  // Voer de regeling alleen uit als de sampleTime is verstreken
  if (currentMillis - lastMillis >= sampleTime) {
    double dt = (double)(currentMillis - lastMillis) / 1000.0; // Werkelijke tijdstap in seconden
    lastMillis = currentMillis;

    Serial.print("Target:");
    Serial.print(targetRPM);
    Serial.print(" ");

    for (int i = 0; i < 4; i++) {
      // 1. Lees de encoder uit en reset direct
      long pos = encoders[i]->read() * encoderDirection[i]; 
      encoders[i]->write(0);
      
      // 2. Bereken actuele RPM
      // Formule: (ticks / CPR) * (60s / dt)
      double currentRPM = ((double)pos / (double)CPR) * (60.0 / dt);

      // 3. PI Regeling
      double error = targetRPM - currentRPM;
      
      // Integraal opbouwen (met anti-windup limit om "op hol slaan" te voorkomen)
      integralError[i] += error * dt;
      integralError[i] = constrain(integralError[i], -100, 100); 

      // Bereken de output (PWM waarde)
      double output = (Kp * error) + (Ki * integralError[i]);
      
      // Begrens de output naar geldige PWM waarden (0-255)
      int finalPWM = constrain((int)output, 0, 255);

      // 4. Stuur de motor aan
      motors[i]->setSpeed(finalPWM);

      // 5. Plotter output voor deze motor
      Serial.print("M");
      Serial.print(i + 1);
      Serial.print(":");
      Serial.print(currentRPM);
      
      if (i < 3) Serial.print(" ");
    }
    Serial.println(); // Sluit de regel af voor de plotter
  }
}