#include "Servo-LX16A.h"

#define GET_LOW_BYTE(A) (uint8_t)((A))
//Macro function  get lower 8 bits of A
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//Macro function  get higher 8 bits of A
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//put A as higher 8 bits   B as lower 8 bits   which amalgamated into 16 bits integer

#define ROVER_SERVO_FRAME_HEADER         0x55
#define ROVER_SERVO_MOVE_TIME_WRITE      1
#define ROVER_SERVO_MOVE_TIME_READ       2
#define ROVER_SERVO_MOVE_TIME_WAIT_WRITE 7
#define ROVER_SERVO_MOVE_TIME_WAIT_READ  8
#define ROVER_SERVO_MOVE_START           11
#define ROVER_SERVO_MOVE_STOP            12
#define ROVER_SERVO_ID_WRITE             13
#define ROVER_SERVO_ID_READ              14
#define ROVER_SERVO_ANGLE_OFFSET_ADJUST  17
#define ROVER_SERVO_ANGLE_OFFSET_WRITE   18
#define ROVER_SERVO_ANGLE_OFFSET_READ    19
#define ROVER_SERVO_ANGLE_LIMIT_WRITE    20
#define ROVER_SERVO_ANGLE_LIMIT_READ     21
#define ROVER_SERVO_VIN_LIMIT_WRITE      22
#define ROVER_SERVO_VIN_LIMIT_READ       23
#define ROVER_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define ROVER_SERVO_TEMP_MAX_LIMIT_READ  25
#define ROVER_SERVO_TEMP_READ            26
#define ROVER_SERVO_VIN_READ             27
#define ROVER_SERVO_POS_READ             28
#define ROVER_SERVO_OR_MOTOR_MODE_WRITE  29
#define ROVER_SERVO_OR_MOTOR_MODE_READ   30
#define ROVER_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define ROVER_SERVO_LOAD_OR_UNLOAD_READ  32
#define ROVER_SERVO_LED_CTRL_WRITE       33
#define ROVER_SERVO_LED_CTRL_READ        34
#define ROVER_SERVO_LED_ERROR_WRITE      35
#define ROVER_SERVO_LED_ERROR_READ       36

//#define ROVER_DEBUG 1  /*Debug ：print debug value*/

byte LX16A::RoverCheckSum(byte buf[])
{
  byte i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (byte)temp;
  return i;
}

void LX16A::RoverSerialServoMove(HardwareSerial &SerialX, uint8_t id, int16_t position, uint16_t time)
{
  byte buf[10];
  if(position < 0)
    position = 0;
  if(position > 1000)
    position = 1000;
  buf[0] = buf[1] = ROVER_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = ROVER_SERVO_MOVE_TIME_WRITE;
  buf[5] = GET_LOW_BYTE(position);
  buf[6] = GET_HIGH_BYTE(position);
  buf[7] = GET_LOW_BYTE(time);
  buf[8] = GET_HIGH_BYTE(time);
  buf[9] = RoverCheckSum(buf);
  SerialX.write(buf, 10);
}

void LX16A::RoverSerialServoStopMove(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[6];
  buf[0] = buf[1] = ROVER_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = ROVER_SERVO_MOVE_STOP;
  buf[5] = RoverCheckSum(buf);
  SerialX.write(buf, 6);
}

void LX16A::RoverSerialServoSetID(HardwareSerial &SerialX, uint8_t oldID, uint8_t newID)
{
  byte buf[7];
  buf[0] = buf[1] = ROVER_SERVO_FRAME_HEADER;
  buf[2] = oldID;
  buf[3] = 4;
  buf[4] = ROVER_SERVO_ID_WRITE;
  buf[5] = newID;
  buf[6] = RoverCheckSum(buf);
  SerialX.write(buf, 7);
  
#ifdef ROVER_DEBUG
  Serial.println("ROVER SERVO ID WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

}

void LX16A::RoverSerialServoSetMode(HardwareSerial &SerialX, uint8_t id, uint8_t Mode, int16_t Speed)
{
  byte buf[10];

  buf[0] = buf[1] = ROVER_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 7;
  buf[4] = ROVER_SERVO_OR_MOTOR_MODE_WRITE;
  buf[5] = Mode;
  buf[6] = 0;
  buf[7] = GET_LOW_BYTE((uint16_t)Speed);
  buf[8] = GET_HIGH_BYTE((uint16_t)Speed);
  buf[9] = RoverCheckSum(buf);

#ifdef ROVER_DEBUG
  Serial.println("ROVER SERVO Set Mode");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  SerialX.write(buf, 10);
}
void LX16A::RoverSerialServoLoad(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = ROVER_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = ROVER_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 1;
  buf[6] = RoverCheckSum(buf);
  
  SerialX.write(buf, 7);
  
#ifdef ROVER_DEBUG
  Serial.println("ROVER SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

}

void LX16A::RoverSerialServoUnload(HardwareSerial &SerialX, uint8_t id)
{
  byte buf[7];
  buf[0] = buf[1] = ROVER_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 4;
  buf[4] = ROVER_SERVO_LOAD_OR_UNLOAD_WRITE;
  buf[5] = 0;
  buf[6] = RoverCheckSum(buf);
  
  SerialX.write(buf, 7);
  
#ifdef ROVER_DEBUG
  Serial.println("ROVER SERVO LOAD WRITE");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif
}

int LX16A::RoverSerialServoReceiveHandle(HardwareSerial &SerialX, byte *ret)
{
  bool frameStarted = false;
  bool receiveFinished = false;
  byte frameCount = 0;
  byte dataCount = 0;
  byte dataLength = 2;
  byte rxBuf;
  byte recvBuf[32];
  byte i;

  while (SerialX.available()) {
    rxBuf = SerialX.read();
    delayMicroseconds(100);
    if (!frameStarted) {
      if (rxBuf == ROVER_SERVO_FRAME_HEADER) {
        frameCount++;
        if (frameCount == 2) {
          frameCount = 0;
          frameStarted = true;
          dataCount = 1;
        }
      }
      else {
        frameStarted = false;
        dataCount = 0;
        frameCount = 0;
      }
    }
    if (frameStarted) {
      recvBuf[dataCount] = (uint8_t)rxBuf;
      if (dataCount == 3) {
        dataLength = recvBuf[dataCount];
        if (dataLength < 3 || dataCount > 7) {
          dataLength = 2;
          frameStarted = false;
        }
      }
      dataCount++;
      if (dataCount == dataLength + 3) {
        
#ifdef ROVER_DEBUG
        Serial.print("RECEIVE DATA:");
        for (i = 0; i < dataCount; i++) {
          Serial.print(recvBuf[i], HEX);
          Serial.print(":");
        }
        Serial.println(" ");
#endif

        if (RoverCheckSum(recvBuf) == recvBuf[dataCount - 1]) {
          
#ifdef ROVER_DEBUG
          Serial.println("Check SUM OK!!");
          Serial.println("");
#endif

          frameStarted = false;
          memcpy(ret, recvBuf + 4, dataLength);
          return 1;
        }
        return -1;
      }
    }
  }
}


int LX16A::RoverSerialServoReadPosition(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = ROVER_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = ROVER_SERVO_POS_READ;
  buf[5] = RoverCheckSum(buf);

#ifdef ROVER_DEBUG
  Serial.println("ROVER SERVO Pos READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (RoverSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2048;

#ifdef ROVER_DEBUG
  Serial.println(ret);
#endif
  return ret;
}

int LX16A::RoverSerialServoReadVin(HardwareSerial &SerialX, uint8_t id)
{
  int count = 10000;
  int ret;
  byte buf[6];

  buf[0] = buf[1] = ROVER_SERVO_FRAME_HEADER;
  buf[2] = id;
  buf[3] = 3;
  buf[4] = ROVER_SERVO_VIN_READ;
  buf[5] = RoverCheckSum(buf);

#ifdef ROVER_DEBUG
  Serial.println("ROVER SERVO VIN READ");
  int debug_value_i = 0;
  for (debug_value_i = 0; debug_value_i < buf[3] + 3; debug_value_i++)
  {
    Serial.print(buf[debug_value_i], HEX);
    Serial.print(":");
  }
  Serial.println(" ");
#endif

  while (SerialX.available())
    SerialX.read();

  SerialX.write(buf, 6);

  while (!SerialX.available()) {
    count -= 1;
    if (count < 0)
      return -2048;
  }

  if (RoverSerialServoReceiveHandle(SerialX, buf) > 0)
    ret = (int16_t)BYTE_TO_HW(buf[2], buf[1]);
  else
    ret = -2049;

#ifdef ROVER_DEBUG
  Serial.println(ret);
#endif
  return ret;
}
