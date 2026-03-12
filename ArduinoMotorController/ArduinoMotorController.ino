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
const unsigned long sampleTime = 100; // Interval in milliseconden

// Correctie voor draairichting encoders per motor
int encoderDirection[4] = {-1, -1, 1, 1}; 

// PID Instellingen
double targetRPM = 60.0; 
double Kp = 1.2;         // Proportioneel: reageert op de huidige fout
double Ki = 0.5;         // Integraal: corrigeert de constante afwijking

double integralError[4] = {0, 0, 0, 0};
double currentPWM[4] = {0, 0, 0, 0};
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
  
  Serial.println("Systeem gestart. Wachten op stabiele loop...");
  lastMillis = millis();
}

void loop() {
  unsigned long currentMillis = millis();

  // Voer de regeling alleen uit als de sampleTime is verstreken
  if (currentMillis - lastMillis >= sampleTime) {
    lastMillis = currentMillis;

    Serial.print("Target:");
    Serial.print(targetRPM);
    Serial.print(" ");

    for (int i = 0; i < 4; i++) {
      // 1. Lees de encoder uit en zet deze direct op 0 voor de volgende meting
      long pos = encoders[i]->read() * encoderDirection[i]; 
      encoders[i]->write(0);
      
      // 2. Bereken actuele RPM
      // Formule: (ticks / CPR) * (60000ms / sampleTime)
      double currentRPM = (double)pos / CPR * (60000.0 / (double)sampleTime);

      // 3. PI Regeling
      double error = targetRPM - currentRPM;
      
      // Integraal opbouwen (met anti-windup limit)
      integralError[i] += error * (sampleTime / 1000.0);
      integralError[i] = constrain(integralError[i], -50, 50); 

      // Bereken de output (Snelheidsverandering)
      double output = (Kp * error) + (Ki * integralError[i]);
      
      // Update de PWM waarde
      currentPWM[i] += output; 
      currentPWM[i] = constrain(currentPWM[i], 0, 255);

      // 4. Stuur de motor aan
      motors[i]->setSpeed((uint8_t)currentPWM[i]);

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