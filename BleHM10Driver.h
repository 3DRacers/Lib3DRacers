#ifndef BLEHM10DRIVER_H
#define BLEHM10DRIVER_H

#include <Arduino.h>
#include <stdlib.h>


class BleHM10Driver
{

public:
	HardwareSerial* BleSerial;
	Serial_* Serial;
	char* baud;
	
	bool isUpdating;
	bool isConnected;

	unsigned int timeout;

	BleHM10Driver(void);
	
	/**
	 * Initialized the module, call it in your setup() function
	 **/	
	void begin(HardwareSerial &bleSerial, Serial_ &serial, byte resetPin);
	
	/**
	 * Initialize the module for incoming connections
	 */
	void initialize();
	
	/**
	 * Receive a structured command, call it in your loop() function
	 **/	
	bool receive(char* input, int size);
	
	/**
	 * Send a structured command
	 **/
	void sendObject(void* ptr, unsigned int objSize);
	
	/**
	 * Send an AT command
	 **/
	void sendCommand(String command);
	
	/**
	 * Reset the module with LOW on RESET pin for 100ms
	 **/
	void reset();
	
	int getBLEName(Stream &ostream, char* ptr, unsigned int objSize);

protected:
	//Net debug:
	unsigned long lastPacketTime;
	unsigned long packetFragmentCount;
	byte bleResetPin;

	bool setBaud(long baud, char* newBaud);

	/**
	 * Send an AT command and check for execution
	 **/
	bool executeCommand(char* PROGMEM command);
	
	void updateMode();
	
	void printHex(char X);
	
	byte CRC8(const byte *data, byte len);
	
};


#endif