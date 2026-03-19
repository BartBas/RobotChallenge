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
const unsigned long sampleTime = 50; // Iets sneller (20Hz) voor betere respons

// Hardware correctie: aanpassen op basis van jouw bedrading
int encoderDirection[4] = {-1, -1, 1, 1}; 

// PID Instellingen (nu arrays per wiel voor onafhankelijke sturing)
double targetRPMs[4] = {0.0, 0.0, 0.0, 0.0}; // Individuele doelen
double integralError[4] = {0, 0, 0, 0};
double Kp = 2.5;
double Ki = 5.0; // Iets verhoogd voor snellere correctie bij lage toeren

unsigned long lastMillis = 0;

void setup() {
  AFMS.begin(50);
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    motors[i] = AFMS.getMotor(i + 1);
    encoders[i] = new QGPMaker_Encoder(i + 1);
    motors[i]->setSpeed(0);
    motors[i]->run(RELEASE);
  }
  
  Serial.println("Slave Systeem Online. Wacht op commando's...");
  lastMillis = millis();
}

void loop() {
  // HIER: Voeg later je Slave-logica toe (bijv. Wire.onReceive of Serial input)
  // Voor nu als voorbeeld: zet hier handmatig targets om te testen
  targetRPMs[0] = -60.0;  // Wiel 1 (rechtsachter)
  targetRPMs[1] = -60.0; // Wiel 2 (rechtsvoor)
  targetRPMs[2] = 60.0;  // Wiel 3 (linksvoor)
  targetRPMs[3] = 60.0; // Wiel 4 (linksachter)

  unsigned long currentMillis = millis();

  if (currentMillis - lastMillis >= sampleTime) {
    double dt = (double)(currentMillis - lastMillis) / 1000.0;
    lastMillis = currentMillis;

    for (int i = 0; i < 4; i++) {
      // 1. Lees de encoder (met richtingcorrectie)
      long pos = encoders[i]->read() * encoderDirection[i]; 
      encoders[i]->write(0);
      
      // 2. Bereken actuele RPM (Snelheid + Richting)
      double currentRPM = ((double)pos / (double)CPR) * (60.0 / dt);

      // 3. PI Regeling
      double error = targetRPMs[i] - currentRPM;
      
      // Integrale actie & Anti-windup
      if (abs(targetRPMs[i]) < 1.0) {
        integralError[i] = 0;
      } else {
        integralError[i] += error * dt;
        integralError[i] = constrain(integralError[i], -150, 150); 
      }

      double output = (Kp * error) + (Ki * integralError[i]);
      
      // 4. Richting en PWM bepalen
      // We gebruiken de absolute waarde voor de PWM, de 'sign' voor de richting
      int finalPWM = constrain(abs((int)output), 0, 255);

      if (targetRPMs[i] > 0) {
        motors[i]->run(FORWARD);
      } else if (targetRPMs[i] < 0) {
        motors[i]->run(BACKWARD);
      } else {
        motors[i]->run(RELEASE);
        finalPWM = 0;
      }

      motors[i]->setSpeed(finalPWM);

      // Debugging/Plotter
      Serial.print("M"); Serial.print(i + 1); Serial.print(":");
      Serial.print(currentRPM); Serial.print(" ");
    }
    Serial.println();
  }
}