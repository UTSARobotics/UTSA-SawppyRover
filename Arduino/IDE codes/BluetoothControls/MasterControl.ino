#include <SoftwareSerial.h>

SoftwareSerial BTserial(1, 0); // Pins of HC-05 - RX | TX

int joystickX = A0; // Analog pin for joystick X axis
int joystickY = A1; // Analog pin for joystick Y axis

void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud rate
  BTserial.begin(38400); // Initialize Bluetooth serial communication at 38400 baud rate
}

void loop() {
  // Read joystick input and map it to a range of -100 to 100
  int x = analogRead(joystickX);
  int y = analogRead(joystickY);

  // Create a comma-seperated string with joystick data
  String RoverControlData = String(x) + "," + String(y);

  // Output joystick input to the Bluetooth module
  BTserial.println(RoverControlData);

  // Serial read for debugging
  Serial.println(RoverControlData);

  // Wait a short period before reading input again
  delay(50);
}
