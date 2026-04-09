#include <Wire.h>
#include "QGPMaker_MotorShield.h"
#include "QGPMaker_Encoder.h"

// Initialize the Shield
QGPMaker_MotorShield AFMS = QGPMaker_MotorShield();
QGPMaker_DCMotor *motors[4];
QGPMaker_Encoder *encoders[4];

// Using the Shield-specific Servo class
QGPMaker_Servo *pickupServo;

// --- CONFIGURATIE ---
const int PPR = 12;
const int gearratio = 90;
const int CPR = (PPR * 4) * gearratio;
const unsigned long sampleTime = 50;
const float MAX_RPM_LIMIT = 80.0;

// Servo Settings
const int SERVO_STOP = 80;
const int SERVO_RUN = 65;

// Sync byte for binary protocol
const uint8_t SYNC_BYTE = 0xFF;

int encoderDirection[4] = { -1, -1, 1, 1 };
double targetRPMs[4] = { 0, 0, 0, 0 };
double integralError[4] = { 0, 0, 0, 0 };
double Kp = 2.5;
double Ki = 5.0;
unsigned long lastMillis = 0;

void setup() {
  AFMS.begin(50);
  Serial.begin(115200);

  pickupServo = AFMS.getServo(0);
  pickupServo->writeServo(SERVO_STOP);

  for (int i = 0; i < 4; i++) {
    motors[i] = AFMS.getMotor(i + 1);
    encoders[i] = new QGPMaker_Encoder(i + 1);
    motors[i]->setSpeed(0);
  }
}

uint8_t buffer[4];
int indexPos = 0;

void readSerialData() {
  if (Serial.available()) {
    uint8_t b = Serial.read(); 

    // Packet assembler
    if (indexPos == 0) {
      if (b == 0xFF) {      // sync byte
        buffer[0] = b;
        indexPos = 1;
      }
    } else {
      buffer[indexPos++] = b;
      
      if (indexPos == 4) {
        uint32_t longbuffer = 0;
        
        for (int i = 0; i < 4; i++) {
          char hexbuf[4];
          longbuffer |= ((uint32_t)buffer[i] & 0xFF) << (8 * (3 - i));
        }
        if (longbuffer != 0) {
          char hexbuf[16];
          bool cmd_enable = (longbuffer >> 19) & 0x01;
          int  direction  = (longbuffer >> 10) & 0x1FF;
          int  turn_val   = (longbuffer >> 8)  & 0x03;
          int  speed_val  = (longbuffer >> 1)  & 0x7F;
          bool pickup     = (longbuffer)       & 0x01;

          processCommand(cmd_enable, direction, turn_val, speed_val, pickup);
        }
        indexPos = 0;
      }
    }
  }
}

void processCommand(bool cmd_enable, int direction, int turn_val, int speed_val, bool pickup) {
  if (cmd_enable && pickup) {
    pickupServo->writeServo(SERVO_RUN);
  } else {
    pickupServo->writeServo(SERVO_STOP);
  }

  if (!cmd_enable) {
    stopAllMotors();
  } else {
    if (direction == 0 && turn_val == 0 && speed_val == 0) {
      stopAllMotors();
    } else {
      calculateMecanum(direction, turn_val, speed_val);
    }
  }
}

void calculateMecanum(int degrees, int turn_val, int speed_val) {
  float t = 0;
  float x = 0, y = 0;

  if (speed_val != 0 && degrees != 0) {
    float targetSpeedRPM = (speed_val / 100.0) * MAX_RPM_LIMIT;
    float rad = (degrees * PI) / 180.0;
    x = sin(rad) * targetSpeedRPM;
    y = cos(rad) * targetSpeedRPM;
  }

  if (turn_val != 0) {
    float turnDir = (turn_val == 1) ? -1.0 : 1.0;
    if (speed_val == 0 || degrees == 0) {
      t = turnDir * MAX_RPM_LIMIT * 0.5;
    } else {
      t = turnDir * (speed_val / 100.0) * MAX_RPM_LIMIT * 0.5;
    }
  }

  targetRPMs[0] = -(y + x + t);  // Front-Left  (Inverted)
  targetRPMs[1] =  (y - x - t);  // Front-Right
  targetRPMs[2] = -(y + x - t);  // Rear-Right  (Inverted)
  targetRPMs[3] =  (y - x + t);  // Rear-Left

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

    if (targetRPMs[i] > 0.1) motors[i]->run(FORWARD);
    else if (targetRPMs[i] < -0.1) motors[i]->run(BACKWARD);
    else {
      motors[i]->run(RELEASE);
      finalPWM = 0;
    }
    motors[i]->setSpeed(finalPWM);
  }
}

void loop() {
  readSerialData();
  updatePID();
}