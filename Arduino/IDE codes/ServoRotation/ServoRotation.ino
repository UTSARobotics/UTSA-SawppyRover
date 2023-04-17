#include <Servo-LX16A.h>

#define ID1   1
#define ID2   2
#define ID3   3

LX16A servo = LX16A();

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
}
void loop() {
  servo.RoverSerialServoSetMode(Serial, ID1, 1, 200);
  servo.RoverSerialServoSetMode(Serial, ID2, 1, 0);
  delay(1000);
  servo.RoverSerialServoSetMode(Serial, ID1, 1, 0);
  servo.RoverSerialServoSetMode(Serial, ID2, 1, 500);
  delay(1000);
}