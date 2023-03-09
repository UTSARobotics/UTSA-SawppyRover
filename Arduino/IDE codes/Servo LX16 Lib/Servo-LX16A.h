#ifndef SERVOLX16A_H
#define SERVOLX16A_H

#if(ARDUINO >= 100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

class LX16A
{
  public:

    byte RoverCheckSum(byte buf[]);
    void RoverSerialServoMove(HardwareSerial &SerialX, uint8_t id, int16_t position, uint16_t time);
    void RoverSerialServoStopMove(HardwareSerial &SerialX, uint8_t id);
    void RoverSerialServoSetID(HardwareSerial &SerialX, uint8_t oldID, uint8_t newID);
    void RoverSerialServoSetMode(HardwareSerial &SerialX, uint8_t id, uint8_t Mode, int16_t Speed);
    void RoverSerialServoLoad(HardwareSerial &SerialX, uint8_t id);
    void RoverSerialServoUnload(HardwareSerial &SerialX, uint8_t id);
    int RoverSerialServoReceiveHandle(HardwareSerial &SerialX, byte *ret);
    int RoverSerialServoReadPosition(HardwareSerial &SerialX, uint8_t id);
    int RoverSerialServoReadVin(HardwareSerial &SerialX, uint8_t id);  

    private:

};

#endif