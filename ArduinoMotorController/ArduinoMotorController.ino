#include <Wire.h>
#include <Servo.h>
#include "QGPMaker_MotorShield.h"
#include "QGPMaker_Encoder.h"

QGPMaker_MotorShield AFMS = QGPMaker_MotorShield();
QGPMaker_DCMotor *motors[4];
QGPMaker_Encoder *encoders[4];
Servo pickupServo;

// --- CONFIGURATIE ---
const int PPR = 12;
const int gearratio = 90;
const int CPR = (PPR * 4) * gearratio;
const unsigned long sampleTime = 50;
const float MAX_RPM_LIMIT = 80.0;

// CONTINUOUS SERVO INSTELLINGEN
const int SERVO_PIN = 9;
const int SERVO_STOP = 90;  // Neutraal (stilstand)
const int SERVO_RUN = 180;  // Voluit draaien

int encoderDirection[4] = { -1, -1, 1, 1 };
double targetRPMs[4] = { 0, 0, 0, 0 };
double integralError[4] = { 0, 0, 0, 0 };
double Kp = 2.5;
double Ki = 5.0;

unsigned long lastMillis = 0;

void setup() {
  AFMS.begin(50);
  Serial.begin(115200);
  
  // Servo initialisatie
  pickupServo.attach(SERVO_PIN);
  pickupServo.write(SERVO_STOP);

  for (int i = 0; i < 4; i++) {
    motors[i] = AFMS.getMotor(i + 1);
    encoders[i] = new QGPMaker_Encoder(i + 1);
    motors[i]->setSpeed(0);
  }
  Serial.println("System Ready. Waiting for 24-bit packets...");
}

void loop() {
  readSerialData();
  updatePID();
}

void readSerialData() {
  // Controleer op 3 bytes (24 bits)
  if (Serial.available() >= 3) {
    uint32_t packet = 0;
    packet |= (uint32_t)Serial.read() << 16;
    packet |= (uint32_t)Serial.read() << 8;
    packet |= (uint32_t)Serial.read();

    // Bits uitpakken
    bool cmd_enable = (packet >> 19) & 0x01;
    int direction   = (packet >> 10) & 0x1FF; // 0-360 graden
    int turn_val    = (packet >> 8)  & 0x03;  // 0=stop, 1=links, 2=rechts
    int speed_val   = (packet >> 1)  & 0x7F;  // 0-100%
    bool pickup     = (packet)       & 0x01;  // Bit 0: Servo

    // Servo aansturing (werkt onafhankelijk van beweging, zolang enable aan is)
    if (cmd_enable && pickup) {
      pickupServo.write(SERVO_RUN);
    } else {
      pickupServo.write(SERVO_STOP);
    }

    // Als enable uit staat: alles uit en stop
    if (!cmd_enable) {
      stopAllMotors();
      return;
    }

    calculateMecanum(direction, turn_val, speed_val);
  }
}

void calculateMecanum(int degrees, int turn_val, int speed_val) {
  // NIEUW: Als er geen richting en geen draaiing is, negeer snelheid en zet alles op 0
  if (degrees == 0 && turn_val == 0) {
    for (int i = 0; i < 4; i++) targetRPMs[i] = 0;
    return;
  }

  float targetSpeedRPM = (speed_val / 100.0) * MAX_RPM_LIMIT;
  float x = 0, y = 0, t = 0;

  // Bereken X en Y componenten
  float rad = (degrees * PI) / 180.0;
  x = cos(rad) * targetSpeedRPM;
  y = sin(rad) * targetSpeedRPM;

  // Bereken Rotatie (Turn)
  if (turn_val == 1)      t = -targetSpeedRPM; // Spin Links
  else if (turn_val == 2) t = targetSpeedRPM;  // Spin Rechts

  // Mecanum Kinematics
  targetRPMs[3] = x + y + t;  // FL
  targetRPMs[0] = x - y - t;  // FR
  targetRPMs[2] = x - y + t;  // RL
  targetRPMs[1] = x + y - t;  // RR

  // Schalen zodat we nooit boven MAX_RPM_LIMIT uitkomen
  float maxVal = 0;
  for (int i = 0; i < 4; i++) {
    if (abs(targetRPMs[i]) > maxVal) maxVal = abs(targetRPMs[i]);
  }
  if (maxVal > MAX_RPM_LIMIT) {
    for (int i = 0; i < 4; i++) {
      targetRPMs[i] = (targetRPMs[i] / maxVal) * MAX_RPM_LIMIT;
    }
  }
}

void stopAllMotors() {
  for (int i = 0; i < 4; i++) {
    targetRPMs[i] = 0;
    integralError[i] = 0;
    motors[i]->setSpeed(0);
    motors[i]->run(RELEASE);
  }
}

void updatePID() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastMillis < sampleTime) return;

  double dt = (double)(currentMillis - lastMillis) / 1000.0;
  lastMillis = currentMillis;

  for (int i = 0; i < 4; i++) {
    long pos = encoders[i]->read() * encoderDirection[i];
    encoders[i]->write(0);

    double currentRPM = ((double)pos / (double)CPR) * (60.0 / dt);
    double error = targetRPMs[i] - currentRPM;

    if (abs(targetRPMs[i]) < 0.5) {
      integralError[i] = 0;
    } else {
      integralError[i] += error * dt;
      integralError[i] = constrain(integralError[i], -150, 150);
    }

    double output = (Kp * error) + (Ki * integralError[i]);
    int finalPWM = constrain(abs((int)output), 0, 255);

    if (targetRPMs[i] > 0.1) {
      motors[i]->run(FORWARD);
    } else if (targetRPMs[i] < -0.1) {
      motors[i]->run(BACKWARD);
    } else {
      motors[i]->run(RELEASE);
      finalPWM = 0;
    }

    motors[i]->setSpeed(finalPWM);
  }
}