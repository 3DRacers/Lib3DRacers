#ifndef ACCEL_COMMAND
#define ACCEL_COMMAND

//Define a packet to be sent to the iOS/Android app:
struct __attribute__ ((packed)) AccelCommand //19bytes
{
  char id;
  unsigned int packetCount;
  byte crc;

  uint8_t teapotPacket[8] = { 0, 0, 0, 0, 0, 0, 0, 0};

  AccelCommand()
  {
    id = 101;
    packetCount = 0;
    crc = 0x00;
  }

};
typedef struct AccelCommand AccelCommand;
#endif
