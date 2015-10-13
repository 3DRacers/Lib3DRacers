#include "Arduino.h"
 
#ifndef STREAMSEND_H
#define STREAMSEND_H
 
 
#define PACKET_NOT_FOUND 0
#define BAD_PACKET 1
#define GOOD_PACKET 2
 
// Set the Max size of the Serial Buffer or the amount of data you want to send+2
// You need to add 2 to allow the prefix and suffix character space to send.
#define MAX_SIZE 64
 
 
class StreamSend {
  private:
    static int getWrapperSize() { return sizeof(char)*2; }
    static byte receiveObject(Stream &ostream, void* ptr, unsigned int objSize, unsigned int loopSize);
    static byte receiveObject(Stream &ostream, void* ptr, unsigned int objSize, unsigned int loopSize, char prefixChar, char suffixChar);
    static char _prefixChar; // Default value is s
    static char _suffixChar; // Default value is e
    static int _maxLoopsToWait;
 
  public:
    static void sendObject(Stream &ostream, void* ptr, unsigned int objSize);
    static void sendObject(Stream &ostream, void* ptr, unsigned int objSize, char prefixChar, char suffixChar);
    static byte receiveObject(Stream &ostream, void* ptr, unsigned int objSize);
    static byte receiveObject(Stream &ostream, void* ptr, unsigned int objSize, char prefixChar, char suffixChar);
    static boolean isPacketNotFound(const byte packetStatus);
    static boolean isPacketCorrupt(const byte packetStatus);
    static boolean isPacketGood(const byte packetStatus);
 
    static void setPrefixChar(const char value) { _prefixChar = value; }
    static void setSuffixChar(const char value) { _suffixChar = value; }
    static void setMaxLoopsToWait(const int value) { _maxLoopsToWait = value; }
    static const char getPrefixChar() { return _prefixChar; }
    static const char getSuffixChar() { return _suffixChar; }
    static const int getMaxLoopsToWait() { return _maxLoopsToWait; }
 
};
 
//Preset Some Default Variables
//Can be modified when seen fit
char StreamSend::_prefixChar = 's';   // Starting Character before sending any data across the Serial
char StreamSend::_suffixChar = 'e';   // Ending character after all the data is sent
int StreamSend::_maxLoopsToWait = -1; //Set to -1 for size of current Object and wrapper
 
 
 
/**
  * sendObject
  *
  * Converts the Object to bytes and sends it to the stream
  *
  * @param Stream to send data to
  * @param ptr to struct to fill
  * @param size of struct
  * @param character to send before the data stream (optional)
  * @param character to send after the data stream (optional)
  */
void StreamSend::sendObject(Stream &ostream, void* ptr, unsigned int objSize) {
  sendObject(ostream, ptr, objSize, _prefixChar, _suffixChar);
}
 
void StreamSend::sendObject(Stream &ostream, void* ptr, unsigned int objSize, char prefixChar, char suffixChar) {
  if(MAX_SIZE >= objSize+getWrapperSize()) { //make sure the object isn't too large
    byte * b = (byte *) ptr; // Create a ptr array of the bytes to send
    ostream.write((byte)prefixChar); // Write the suffix character to signify the start of a stream
 
    // Loop through all the bytes being send and write them to the stream
    for(unsigned int i = 0; i<objSize; i++) {
      ostream.write(b[i]); // Write each byte to the stream
    }
    ostream.write((byte)suffixChar); // Write the prefix character to signify the end of a stream
  }
}
 
/**
  * receiveObject
  *
  * Gets the data from the stream and stores to supplied object
  *
  * @param Stream to read data from
  * @param ptr to struct to fill
  * @param size of struct
  * @param character to send before the data stream (optional)
  * @param character to send after the data stream (optional)
  */
byte StreamSend::receiveObject(Stream &ostream, void* ptr, unsigned int objSize) {
    return receiveObject(ostream, ptr, objSize, _prefixChar, _suffixChar);
}
byte StreamSend::receiveObject(Stream &ostream, void* ptr, unsigned int objSize, char prefixChar, char suffixChar) {
  return receiveObject(ostream, ptr, objSize, 0, prefixChar, suffixChar);
}
 
byte StreamSend::receiveObject(Stream &ostream, void* ptr, unsigned int objSize, unsigned int loopSize, char prefixChar, char suffixChar) {
  int maxLoops = (_maxLoopsToWait == -1) ? (objSize+getWrapperSize()) : _maxLoopsToWait;
  if(loopSize >= maxLoops) {
      return PACKET_NOT_FOUND;
  }
  if(ostream.available() >= (objSize+getWrapperSize())) { // Packet meets minimum size requirement
    if(ostream.read() != (byte)prefixChar) {
      // Prefix character is not found
      // Loop through the code again reading the next char
      return receiveObject(ostream, ptr, objSize, loopSize+1, prefixChar, suffixChar);
    }
 
    char data[objSize]; //Create a tmp char array of the data from Stream
    ostream.readBytes(data, objSize); //Read the # of bytes
    memcpy(ptr, data, objSize); //Copy the bytes into the struct
 
    if(ostream.read() != (byte)suffixChar) {
      //Suffix character is not found
      return BAD_PACKET;
    }
      return GOOD_PACKET;
  }
  return PACKET_NOT_FOUND; //Prefix character wasn't found so no packet detected
}
 
 
boolean StreamSend::isPacketNotFound(const byte packetStatus) {
    return (packetStatus == PACKET_NOT_FOUND);
}
 
boolean StreamSend::isPacketCorrupt(const byte packetStatus) {
    return (packetStatus == BAD_PACKET);
}
 
boolean StreamSend::isPacketGood(const byte packetStatus) {
    return (packetStatus == GOOD_PACKET);
}
 
#endif
/*
 
 
//////////////////////////////////////////////////////////////////////////////////////
//Sending Examples:
//////////////////////////////////////////////////////////////////////////////////////
char charVariable = 'c';
StreamSend::sendObject(Serial, &charVariable, sizeof(charVariable));
StreamSend::sendObject(Serial, &charVariable, sizeof(charVariable), 'a', 'z');
 
 
int intVariable = 13496;
StreamSend::sendObject(xbeeSerial, &intVariable, sizeof(intVariable));
StreamSend::sendObject(xbeeSerial, &intVariable, sizeof(intVariable), 'j', 'p');
 
 
struct SIMPLE_STRUCT {
  char charVariable;
  int intVariable[7];
  boolean boolVariable;
};
SIMPLE_STRUCT simpleStruct;
simpleStruct.charVariable = 'z';
for(int i=0; i<7; i++) {
  simpleStruct.intVariable[i] = i;
}
StreamSend::sendObject(xbeeSerial, &simpleStruct, sizeof(simpleStruct));
StreamSend::sendObject(xbeeSerial, &simpleStruct, sizeof(simpleStruct), '3', 'u');
 
 
 
//////////////////////////////////////////////////////////////////////////////////////
//Receiving Examples:
//////////////////////////////////////////////////////////////////////////////////////
char charVariable;
byte packetResults = StreamSend::receiveObject(Serial, &charVariable, sizeof(charVariable));
byte packetResults = StreamSend::receiveObject(Serial, &charVariable, sizeof(charVariable), 'a', 'z');
 
 
int intVariable;
byte packetResults = StreamSend::receiveObject(xbeeSerial, &intVariable, sizeof(intVariable));
byte packetResults = StreamSend::receiveObject(xbeeSerial, &intVariable, sizeof(intVariable), 'j', 'p');
 
 
struct SIMPLE_STRUCT {
  char charVariable;
  int intVariable[7];
  boolean boolVariable;
};
SIMPLE_STRUCT simpleStruct;
byte packetResults = StreamSend::receiveObject(xbeeSerial, &simpleStruct, sizeof(simpleStruct));
byte packetResults = StreamSend::receiveObject(xbeeSerial, &simpleStruct, sizeof(simpleStruct), '3', 'p');
 
 
//////////////////////////////////////////////////////////////////////////////////////
//Test To see if Packet was Valid:
//////////////////////////////////////////////////////////////////////////////////////
 
if(StreamSend::isPacketGood(packetResults)) {
  //The Packet was Good
} else {
  //The Packet was Bad
}
 
if(StreamSend::isPacketCorrupt(packetResults)) {
  //The Packet was Corrupt
} else {
  //The Packet wasn't found or it was Good
}
 
if(StreamSend::isPacketNotFound(packetResults)) {
  //The Packet was not found after Max # of Tries
} else {
  //The Packet was Found, but can be corrupt
}
 
*/
