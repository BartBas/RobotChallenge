#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// ==========================================
// CONFIGURATIE
// ==========================================
const long BAUDRATE = 115200;       // Zorg dat je Python script dit ook gebruikt!
const int TIMEOUT_MS = 500;         // Veiligheid: stop na 0.5s geen signaal
const float DEG_TO_RAD = 0.01745329251; 
const int MAX_PWM = 4095;           // PCA9685 max waarde

// Initialiseer de PWM driver op adres 0x60
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x60);

// --- PIN DEFINITIES (PCA9685) ---
// Motor 1 (Rechts Achter)
const int M1_IN1 = 8;
const int M1_IN2 = 9;
// Motor 3 (Rechts Voor)
const int M3_IN1 = 10;
const int M3_IN2 = 11;
// Motor 2 (Links Achter)
const int M2_IN1 = 12;
const int M2_IN2 = 13;
// Motor 4 (Links Voor)
const int M4_IN1 = 14;
const int M4_IN2 = 15;

// Variabele voor de watchdog timer
unsigned long lastCommandTime = 0; 

// --- FUNCTIE PROTOTYPES ---
void setMotor(int motorID, int speed); 
void StopMotors();

// ==========================================
// 1. HARDWARE AANSTURING & MAPPING
// ==========================================

// Deze functie koppelt de berekende waarden aan de juiste motoren.
// Omdat +4095 bij jou altijd vooruit is, hoeven we hier GEEN mintekens te gebruiken.
void SetAllMotors(int fl, int fr, int bl, int br) {
  
  // Mapping gebaseerd op jouw info:
  // Motor 4 = Links Voor
  // Motor 3 = Rechts Voor
  // Motor 2 = Links Achter
  // Motor 1 = Rechts Achter

  setMotor(4, fl);      // Links Voor
  setMotor(3, fr);      // Rechts Voor 
  setMotor(2, bl);      // Links Achter
  setMotor(1, br);      // Rechts Achter
}

void StopMotors() {
  setMotor(1, 0);
  setMotor(2, 0);
  setMotor(3, 0);
  setMotor(4, 0);
}

// Jouw originele driver functie
void setMotor(int motorID, int speed) {
  int pinA, pinB;

  switch (motorID) {
    case 1: pinA = M1_IN1; pinB = M1_IN2; break;
    case 2: pinA = M2_IN1; pinB = M2_IN2; break;
    case 3: pinA = M3_IN1; pinB = M3_IN2; break;
    case 4: pinA = M4_IN1; pinB = M4_IN2; break;
    default: return;
  }

  // Begrens de snelheid voor de zekerheid
  if (speed > 4095) speed = 4095;
  if (speed < -4095) speed = -4095;

  if (speed > 0) {
    // Vooruit
    pwm.setPWM(pinA, 0, speed);
    pwm.setPWM(pinB, 0, 0);    
  } else if (speed < 0) {
    // Achteruit
    pwm.setPWM(pinA, 0, 0);    
    pwm.setPWM(pinB, 0, -speed); // Maak positief voor de PWM waarde
  } else {
    // Stop
    pwm.setPWM(pinA, 0, 0);
    pwm.setPWM(pinB, 0, 0);
  }
}

// ==========================================
// 2. REKENHART (LOGICA)
// ==========================================

void ProcessCommand(int graden, int turn, int speed) {
    lastCommandTime = millis(); // Reset timeout

    float speedFactor = speed / 100.0;
    float x = 0;
    float y = 0;
    float rot = 0;

    // A. TRANSLATIE
    // 0 graden = STIL, 360 graden = VOORUIT
    if (graden == 0) {
        x = 0; 
        y = 0;
    } else {
        float rad = graden * DEG_TO_RAD;
        y = cos(rad) * speedFactor; // Vooruit
        x = sin(rad) * speedFactor; // Opzij
    }

    // B. ROTATIE
    if (turn == 1) rot = -speedFactor; // Links
    if (turn == 2) rot = speedFactor;  // Rechts

    // C. MECANUM MIXING FORMULE
    // Hier ontstaan vanzelf negatieve waarden als dat nodig is voor de beweging
    float fl = y + x + rot;
    float fr = y - x - rot;
    float bl = y - x + rot;
    float br = y + x - rot;

    // D. NORMALISEREN
    float maxVal = max(abs(fl), max(abs(fr), max(abs(bl), abs(br))));
    if (maxVal > 1.0) {
        fl /= maxVal;
        fr /= maxVal;
        bl /= maxVal;
        br /= maxVal;
    }

    // E. STUUR NAAR HARDWARE
    SetAllMotors(fl * MAX_PWM, fr * MAX_PWM, bl * MAX_PWM, br * MAX_PWM);
}

// ==========================================
// 3. PROTOCOL PARSING
// ==========================================

void ParseBytes(uint8_t b1, uint8_t b2, uint8_t b3) {
    uint32_t combined = ((uint32_t)b1 << 16) | ((uint32_t)b2 << 8) | b3;

    // [CMD 1][Graden 9][Turn 2][Speed 7][Padding 5]
    int graden = (combined >> 14) & 0x1FF;
    int turn   = (combined >> 12) & 0x03;
    int speed  = (combined >> 5)  & 0x7F;

    if (speed > 100) speed = 100;
    if (graden > 360) graden = 360; 

    ProcessCommand(graden, turn, speed);
}

// ==========================================
// 4. MAIN SETUP & LOOP
// ==========================================

void setup() {
  Serial.begin(BAUDRATE); // 115200 baud
  
  pwm.begin();
  pwm.setPWMFreq(1000); // 1000Hz frequentie

  StopMotors();
  // Serial.println("Robot Ready."); 
  // Let op: Printen kan je protocol verstoren als de Pi ook luistert!
}

void loop() {
  // 1. Check Serial Buffer
  if (Serial.available() >= 4) {
    if (Serial.peek() != -1) {
      uint8_t buffer[4];
      Serial.readBytes(buffer, 4);

      if (buffer[3] == '\r') {
        ParseBytes(buffer[0], buffer[1], buffer[2]);
      } else {
        // Sync error handling
        while(Serial.available() && Serial.peek() != '\r') {
          Serial.read();
        }
        Serial.read(); 
      }
    }
  }

  // 2. Watchdog Timer
  if (millis() - lastCommandTime > TIMEOUT_MS) {
    StopMotors();
  }
}