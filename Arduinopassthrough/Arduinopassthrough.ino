void setup() {
  // To PC (Programming Port)
  Serial.begin(115200); 
  
  // To Raspberry Pi (Native USB Port)
  SerialUSB.begin(115200); 
  
  // Wait for the Native port to be ready
  while(!SerialUSB); 
}

void loop() {
  // Forward from Pi (Native) to PC (Programming)
  if (SerialUSB.available()) {
    byte b = SerialUSB.read();
    Serial.write(b);
  }

  // Forward from PC (Programming) to Pi (Native)
  if (Serial.available()) {
    byte b = Serial.read();
    SerialUSB.write(b);
  }
}