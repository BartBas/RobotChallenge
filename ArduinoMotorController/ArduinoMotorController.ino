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
const int SERVO_RUN = 70;

int encoderDirection[4] = { -1, -1, 1, 1 }; 
double targetRPMs[4] = { 0, 0, 0, 0 }; 
double integralError[4] = { 0, 0, 0, 0 }; 
double Kp = 2.5; 
double Ki = 5.0; 
unsigned long lastMillis = 0; 

void setup() {
  // Start the shield at 50Hz (Standard for Servos)
  AFMS.begin(50); 
  Serial.begin(115200); 
  
  // Initialize Servo Header 0
  pickupServo = AFMS.getServo(0); 
  pickupServo->writeServo(SERVO_STOP); 

  for (int i = 0; i < 4; i++) { 
    motors[i] = AFMS.getMotor(i + 1); 
    encoders[i] = new QGPMaker_Encoder(i + 1); 
    motors[i]->setSpeed(0); 
  }
  
  Serial.println("--- Systeem Gereed ---");
  Serial.println("Type commando als: enable,direction,turn,speed,pickup"); 
}

void loop() {
  readSerialData(); 
  updatePID(); 
}

void readSerialData() {
  if (Serial.available() > 0) {
    // We kijken naar het eerste byte zonder het te verwijderen ('peek')
    char firstByte = Serial.peek();

    // Methode 1: Tekstgebaseerde data (begint vaak met een cijfer 0-9)
    if (isDigit(firstByte) || firstByte == '-') {
      String input = Serial.readStringUntil('\n');
      input.trim();

      if (input.length() > 0) {
        int values[5];
        int count = 0;
        int startPos = 0;
        int commaPos = input.indexOf(',');

        while (commaPos != -1 && count < 4) {
          values[count++] = input.substring(startPos, commaPos).toInt();
          startPos = commaPos + 1;
          commaPos = input.indexOf(',', startPos);
        }
        values[count] = input.substring(startPos).toInt();

        // Verwerk de waarden
        processCommand(values[0], values[1], values[2], values[3], values[4]);
      }
    } 
    // Methode 2: Binair pakket (3 bytes)
    else if (Serial.available() >= 3) {
      uint32_t packet = 0;
      packet |= (uint32_t)Serial.read() << 16;
      packet |= (uint32_t)Serial.read() << 8;
      packet |= (uint32_t)Serial.read();

      // Bits uitpakken
      bool cmd_enable = (packet >> 19) & 0x01;
      int direction   = (packet >> 10) & 0x1FF; // 0-360 graden
      int turn_val    = (packet >> 8)  & 0x03;  // 0=stop, 1=links, 2=rechts
      int speed_val   = (packet >> 1)  & 0x7F;  // 0-100%
      bool pickup     = (packet)       & 0x01;  // Bit 0

      // Verwerk de waarden
      processCommand(cmd_enable, direction, turn_val, speed_val, pickup);
    }
    else {
      // Als er iets anders binnenkomt dat we niet herkennen, 
      // gooien we 1 byte weg om de buffer schoon te houden.
      Serial.read();
    }
  }
}

void processCommand(bool cmd_enable, int direction, int turn_val, int speed_val, bool pickup) {
  // Servo aansturing met de juiste pointer-notatie (->)
  if (cmd_enable && pickup) {
    pickupServo->writeServo(SERVO_RUN); 
  } else {
    pickupServo->writeServo(SERVO_STOP);
  }

  // Motor aansturing
  if (!cmd_enable) {
    stopAllMotors();
  } else {
    calculateMecanum(direction, turn_val, speed_val);
  }
}

void calculateMecanum(int degrees, int turn_val, int speed_val) {
  if (degrees == 0 && turn_val == 0) { 
    for (int i = 0; i < 4; i++) targetRPMs[i] = 0; 
    return;
  }

  float targetSpeedRPM = (speed_val / 100.0) * MAX_RPM_LIMIT; 
  float x = 0, y = 0, t = 0;
  float rad = (degrees * PI) / 180.0; 
  
  x = cos(rad) * targetSpeedRPM; 
  y = sin(rad) * targetSpeedRPM; 
  
  if (turn_val == 1)      t = -targetSpeedRPM; 
  else if (turn_val == 2) t = targetSpeedRPM; 

  targetRPMs[3] = x + y + t; 
  targetRPMs[0] = x - y - t; 
  targetRPMs[2] = x - y + t; 
  targetRPMs[1] = x + y - t; 

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

    if (targetRPMs[i] > 0.1)      motors[i]->run(FORWARD); 
    else if (targetRPMs[i] < -0.1) motors[i]->run(BACKWARD); 
    else {
      motors[i]->run(RELEASE); 
      finalPWM = 0;
    }
    motors[i]->setSpeed(finalPWM);
  }
}