#include <SoftwareSerial.h>

SoftwareSerial BTserial(0, 1); // Pins of HC-05 - RX | TX

void setup() {
  Serial.begin(38400); // Initialize serial communication at 38400 baud rate
  BTserial.begin(38400); // Initialize Bluetooth serial communication at 38400 baud rate
}

void loop() {
  if (BTserial.available()) {
    String receivedData = BTserial.readStringUntil('\n');
    // Serial.println("Received data: " + receivedData);

    // Process the received data (e.g., control a motor, display values, etc.)
  }
  
}
