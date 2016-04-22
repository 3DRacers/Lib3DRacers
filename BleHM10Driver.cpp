#ifndef BLEHM10DRIVER_C
#define BLEHM10DRIVER_C

#include "BleHM10Driver.h"
#include "Config.h"

#include <Arduino.h>
#include <stdlib.h>
#include <StreamSend.h>

BleHM10Driver::BleHM10Driver() :
	lastPacketTime(0),
	packetFragmentCount(0),
	isUpdating(false),
	isConnected(false),
	timeout(1000),
	bleResetPin(0)
{
}

void BleHM10Driver::begin(HardwareSerial &bleSerial, Serial_ &serial, byte resetPin) {
	lastPacketTime = millis();
	BleSerial = &bleSerial;
	Serial = &serial;
	bleResetPin = resetPin;
		
	BleSerial->setTimeout(200); //Wait while reding string/bytes up to 200ms, applied to all the find* and read* methods of the Stream	 
	initialize();
}

void BleHM10Driver::initialize() {
	STARTLOGFln("[BLE] Initializing:");
	
	reset();
	
	//Try to communicate with the standard baud rate to set our desired baud:
	//Baud Rate 0---------9600 1---------19200 2---------38400 3---------57600 4---------115200 5---------4800 6---------2400 7---------1200 8---------230400
	STARTLOGF("[BLE] +- Setting baud rate to: ");
	STARTLOG(baud);
	const long allowedBauds[] = { 9600, 19200, 38400, 57600, 115200, 4800, 2400, 1200, 230400 };
	bool baudFound = false;
	for(int c=0; c < 5; c++) {
		if(setBaud(allowedBauds[c], "AT+BAUD3")) {
			baudFound = true;
			STARTLOGFln(" [OK]");
			break;
		}
	}
	if(!baudFound) {
		STARTLOGFln(" [ERR] Not Found!");
		//If BLE module not working, put the Pilot board in update mode. (ie: console is proxied directly to the BLE module)
		updateMode();
		return;
	}
	
	BleSerial->begin(57600);
	
	#if STARTUP_LOG
	BleSerial->print("AT+VERR?");
	STARTLOGF("[BLE] +- Ver: ");
	STARTLOG(BleSerial->readString());	
    STARTLOGFln(" [OK]");
	#endif
	
	executeCommand("AT+MODE0"); //0: Transmission Mode (after connection. act as a transparent serial proxy) 1: PIO collection Mode +Mode 0 2: Remote Control Mode+ Mode 0
	executeCommand("AT+ROLE0"); //0: Peripheral (ie: client, while the remote is the master "Central" node) 1: Central
	executeCommand("AT+FILT0"); //1: Find only HM-10 devices 0: All
	executeCommand("AT+POWE0"); //Power 0 highest, 3 low, default 2 0: -23dbm 1: -6dbm 2: 0dbm [3: 6dbm?] (ps: maybe only with 2540 chipset)
	//Disabled in V526: executeCommand("AT+FIOW0"); //Flow control: 0 disabled, 1 enabled, default 0 (Need RTS and CTS pin connected)
	executeCommand("AT+SHOW1"); //Show name in discovery
	executeCommand("AT+TYPE0"); //No PIN
	
	//V540:
	/*executeCommand("AT+COMI9"); //Minimum Link Layer connection interval 0: 7.5ms; 1: 10ms; 2: 15ms; 3: 20ms; 4: 25ms; 5: 30ms; 6: 35ms; 7: 40ms; 8: 45ms; 9: 4000ms, default: 3
	executeCommand("AT+COMA9"); //Maximum Link Layer connection interval 0: 7.5ms; 1: 10ms; 2: 15ms; 3: 20ms; 4: 25ms; 5: 30ms; 6: 35ms; 7: 40ms; 8: 45ms; 9: 4000ms, default: 7
	executeCommand("AT+COLA4"); //Link Layer connection slave latency 0 ~ 4; Default: 0
	executeCommand("AT+COSU0"); //Link Layer connection supervision timeout  0: 100ms; 1: 1000ms; 2: 2000ms; 3: 3000ms; 4: 4000ms; 5: 5000ms; 6: 6000ms; Default: 6(6000ms);
	executeCommand("AT+COUP1"); //Switch slave role update connection parameter  0, 1; Default: 1(Update on);
	executeCommand("AT+GAIN1"); // 0: No RX gain 1: Open RX gain, default: 0
	*/
	
	//Disabled in V526: executeCommand("AT+GAIN0"); // 0: No RX gain 1: Open RX gain, default: 0
	executeCommand("AT+IMME1"); //0: Start immediately, not used in peripheral mode, 1: Wait for AT+START
	BleSerial->print("AT+START"); //Start connecting, used only with IMME1
	delay(500);
	STARTLOGFln("[BLE] Ready and listening [OK]");
	
	//other:
	//AT+PCTL output PINS driver power: 0:Normal power output 1:Max power output
	
	//NEW in 538:
	/*. 1. Add AT+COMI command, config Minimum Link Layer connection interval
		   para1 value: 0 ~ 9; Default: 3(20ms);
		   0: 7.5ms; 1: 10ms; 2: 15ms; 3: 20ms; 4: 25ms; 5: 30ms; 6: 35ms; 7: 40ms; 8: 45ms; 9: 4000ms
		2. Add AT+COMA command, config Maximum Link Layer connection interval
		   para1 value: 0 ~ 9; Default: 7(40ms);
		   0: 7.5ms; 1: 10ms; 2: 15ms; 3: 20ms; 4: 25ms; 5: 30ms; 6: 35ms; 7: 40ms; 8: 45ms; 9: 4000ms
		3. Add AT+COLA command, config Link Layer connection slave latency
		   para1 value: 0 ~ 4; Default: 0;
		4. Add AT+COSU command, config Link Layer connection supervision timeout
		   para1 value: 0 ~ 6; Default: 6(6000ms);
		   0: 100ms; 1: 1000ms; 2: 2000ms; 3: 3000ms; 4: 4000ms; 5: 5000ms; 6: 6000ms;
		5. Add AT+COUP command, switch slave role update connection parameter
		   para1 value 0, 1; Default: 1(Update on);
		   0: Update off; 1: Update on;*/
		
}

void BleHM10Driver::reset() {
	pinMode(bleResetPin, OUTPUT);
	digitalWrite(bleResetPin, LOW);
	delay(120);
	pinMode(bleResetPin, INPUT);
}

void BleHM10Driver::sendCommand(String command) {
	#if not REDUCED_FEATURES
	if(command == "AT+SBLUP") {
		BleSerial->print(command);
		updateMode();
	}
	else {
		if (isConnected) {
			reset();
			delay(500);
		}
				
		BleSerial->print(command);
		#if SHELL_ENABLED
			SHELLOUTln(BleSerial->readString());
		#endif
	}
	#endif
}

void BleHM10Driver::updateMode() {
	#if not REDUCED_FEATURES
	isUpdating = true;
	SHELLOUTln(F("[BLE] Update mode (115200 baud set). [OK]"));

	//In Update mode the module requires 115200 baud:
	BleSerial->flush();
	delay(50);
	BleSerial->end();
	delay(100);
	BleSerial->begin(115200);

	//And Arduino:
	Serial->flush();
	delay(50);
	Serial->end();
	delay(100);
	Serial->begin(115200);
	#endif
}

bool BleHM10Driver::executeCommand(char* PROGMEM command) {
	STARTLOGF("[BLE] +- ");
	STARTLOG(command);
	BleSerial->print(command); //0: Transmission Mode (after connection. act as a transparent serial proxy) 1: PIO collection Mode +Mode 0 2: Remote Control Mode+ Mode 0
	if(!BleSerial->find("OK")) { 
		STARTLOGFln(" Cmd not recognized! [ERR]"); 
		return false;
	}
	STARTLOGFln(" [OK]");
	return true;
}

//Baud Rate 0---------9600 1---------19200 2---------38400 3---------57600 4---------115200
bool BleHM10Driver::setBaud(long baud, char* newBaud) {
	STARTLOGF(" - ");
	STARTLOG(baud);
	BleSerial->flush();
	delay(50);
	BleSerial->end();
	delay(100);
	BleSerial->begin(baud); //Start with default baud for module
	delay(100);
	//Change BAUD (if already set, ignore it):
	BleSerial->print(newBaud); 
	delay(100);
	if(!BleSerial->find("OK")) { return false; }
	
	STARTLOGF("-> Found!");
	BleSerial->print("AT+RESET");
	
	//Switch to new baud:
	BleSerial->flush();
	delay(50);
	BleSerial->end();
	delay(100);
	return true;
}

bool BleHM10Driver::receive(char* input, int size) {
	bool found = false;

	//Receive command
	if (BleSerial->available() >= size / 2) { //Just check for half the bytes available, when the read occur the other will be there
		
		byte packetResults = StreamSend::receiveObject(*BleSerial, input, size);

		if (StreamSend::isPacketGood(packetResults)) {

			#if SHELL_ENABLED
			if(!isConnected) {
				SHELLOUTFln("[BLE] Connected. [OK]");
			}
			#endif
			isConnected = true;

			#if SERIAL_DEBUG_NET

				long freq = millis() - lastPacketTime;
				DEBUGNET(F("[INFO] [BLE] PKT freq: "));
			
				for(int c=0; c < freq && c < 100; c +=10) {
					DEBUGNET(F("#"));
				}
				for(int c=100; c > freq; c -=10) {
					DEBUGNET(F(" "));
				}
				DEBUGNET(millis() - lastPacketTime);
				DEBUGNET(F("ms - byte:"));
				DEBUGNET(size);
				DEBUGNET(F(" frag:"));
				DEBUGNET(packetFragmentCount);
				DEBUGNET(F(" hex: "));
				for(int c=0; c < size; c++) {
					printHex(input[c]);
					DEBUGNET(F(" "));
				}
				
				DEBUGNETln("");
				packetFragmentCount = 0;

			#endif
			
			lastPacketTime = millis();
			
			found = true;
		}
		else if (StreamSend::isPacketCorrupt(packetResults)) {
			#if SERIAL_DEBUG_NET
				DEBUGNET(F("[BLE] [ERR] (Corrupted PKT)"));
				DEBUGNET(millis() - lastPacketTime);
				DEBUGNET(F("ms - byte:"));
				DEBUGNET(size);
				DEBUGNET(F(" frag:"));
				DEBUGNET(packetFragmentCount);
				DEBUGNET(F(" hex: "));
				for(int c=0; c < size; c++) {
					printHex(input[c]);
					DEBUGNET(F(" "));
				}
				DEBUGNET(F(" - "));
				DEBUGNET(size);
				DEBUGNETln(F(""));
			#endif
		}
		else {
			#if SERIAL_DEBUG_NET
				packetFragmentCount++;
			#endif
		}

	}
	
	if(isConnected && lastPacketTime +timeout < millis()) {
		//NB: during configuration the cars automaticly disconnect due to a timeout (since no DriveCmd packets are sent during the car setup)
		SHELLOUTFln("[BLE] Timeout Disconnected. [ERR]");
		isConnected = false;
	}
	return found;
}

void BleHM10Driver::printHex(char X) {

   if (X < 16) { DEBUGNET("0"); }
   DEBUGNET(X, HEX);

}

void BleHM10Driver::sendObject(void* packet, unsigned int objSize) {
	#if SERIAL_DEBUG_NET
	DEBUGNET(F("[INFO] [BLE] "));
	byte * b = (byte *) packet;
	for(int c=0; c < objSize; c++) {
		printHex(b[c]);
		DEBUGNET(F(" "));
	}
	DEBUGNETln(F(""));		
	#endif
	StreamSend::sendObject(*this->BleSerial, packet, objSize);
}

int BleHM10Driver::getBLEName(Stream &ostream, char* ptr, unsigned int objSize) {
	#if !REDUCED_FEATURES
	//empty receive buffer
	ostream.flush();
	delay(100);
	while (ostream.available() > 0)
	{
		char t = ostream.read();
	}
	
	ostream.print("AT+NAME?"); //Send name request to BLE
	ostream.flush();
	delay(200);
	
	if(!ostream.find("OK+NAME:")) {
		DEBUGln(F("[BLE] Name not found! [ERR]"));
		return 0;
	}
	
	//message is ok. Read name now
	if (ostream.available()){

		char name[14];
		int bytes = ostream.readBytes(name, 13); //12 is max length by HM-10 doc
		if(bytes == 0) return 0;
		
		name[bytes+1] = '\0'; //String terminator
		
		memcpy(ptr, name, objSize); //Copy the bytes into the struct

		return bytes+1;
	}
	#endif
	return 0;
}

#endif