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
	STARTLOGFln("[BLE] Initializing module:");
	
	reset();
	
	//Try to communicate with the standard baud rate to set our desired baud:
	//Baud Rate 0---------9600 1---------19200 2---------38400 3---------57600 4---------115200
	STARTLOGF("[BLE] +- Setting baud rate to: ");
	STARTLOG(baud);
	const long allowedBauds[] = { 9600, 19200, 38400, 57600, 115200 };
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
	}
	BleSerial->begin(57600);
	
	BleSerial->print("AT+VERR?");
	STARTLOGF("[BLE] +- Module version: ");
	STARTLOG(BleSerial->readString());	
    STARTLOGFln(" [OK]");		

	//Change factory name:
	STARTLOGF("[BLE] +- Name: ");
	char name[13];
	int found = getBLEName(*BleSerial, name, sizeof(name));
	if(found > 0) {
		STARTLOG(name);
		STARTLOGFln(" [OK]");
		/* TODO: some memory error causes a crash here:
		const char factoryName[] = { 'H', 'M', 'S', 'o', 'f', 't' };
		/*bool sameName = true;
		for(int c = 0; c < 6; c++) {
			if(name[c] != factoryName[c]) {
				sameName = false;
				break;
			}
		}
		if(sameName) {		
			//Factory reset:
			//BleSerial->print("AT+RENEW");
			//delay(200);
			//TODO: this reset also the baud
			
			//NB: can't use random or snprintf since the sketch size is too big!
			/*char buffer[18];
			for(int c=0; c < 100; c++) {
				analogRead(A0);
			}
			randomSeed(analogRead(A0));
			
			snprintf (buffer, 18, "AT+NAME3DRacers%d", random(10, 99));
			*//*
			BleSerial->print("AT+NAME3DRacers");
			BleSerial->print(analogRead(A0));
			delay(100);
			STARTLOGF("[BLE] +- Default name set.");
			STARTLOGFln(" [OK]");
		}*/
	}
	else {
		STARTLOGFln("[ERR]");
	}
	
	executeCommand("AT+MODE0"); //0: Transmission Mode (after connection. act as a transparent serial proxy) 1: PIO collection Mode +Mode 0 2: Remote Control Mode+ Mode 0
	executeCommand("AT+ROLE0"); //0: Peripheral (ie: client, while the remote is the master "Central" node) 1: Central
	executeCommand("AT+FILT0"); //1: Find only HM-10 devices 0: All
	executeCommand("AT+POWE0"); //Power 0 highest, 3 low, default 2 0: -23dbm 1: -6dbm 2: 0dbm [3: 6dbm?] (ps: maybe only with 2540 chipset)
	//Disabled in V526: executeCommand("AT+FIOW0"); //Flow control: 0 disabled, 1 enabled, default 0 (Need RTS and CTS pin connected)
	executeCommand("AT+SHOW1"); //Show name in discovery
	executeCommand("AT+TYPE0"); //No PIN
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

	if(command == "AT+SBLUP") {
		DEBUGln(F("[BLE] Entering BLE Module update mode [OK]"));
		isUpdating = true;

		DEBUGln(F("[BLE] Now you can close the console and open the HMSoft update tool (115200 baud set). [OK]"));
		executeCommand("AT+SBLUP");

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
	}
	else {
		BleSerial->print(command);
		#if SHELL_ENABLED
			SHELLOUTln(BleSerial->readString());
		#endif
	}
}

bool BleHM10Driver::executeCommand(char* PROGMEM command) {
	STARTLOGF("[BLE] +- ");
	STARTLOG(command);
	BleSerial->print(command); //0: Transmission Mode (after connection. act as a transparent serial proxy) 1: PIO collection Mode +Mode 0 2: Remote Control Mode+ Mode 0
	if(!BleSerial->find("OK")) { 
		STARTLOGFln(" Module doesn't respond or doesn't recognize command! [ERR]"); 
		return false;
	}
	STARTLOGFln(" [OK]");
	return true;
}

//Baud Rate 0---------9600 1---------19200 2---------38400 3---------57600 4---------115200
bool BleHM10Driver::setBaud(long baud, char* newBaud) {
	STARTLOGF(" - ");
	STARTLOG(baud);
	BleSerial->begin(baud); //Start with default baud for module
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
			#if SERIAL_DEBUG_NET
			if(!isConnected) {
				DEBUGln(F("[BLE] Connected. [OK]"));
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
				
			//    //Check for sanity (errors comes from BLE to Arduino communication):
			//    lastCrc = rcv.crc;
			//    rcv.crc = 0x00;
			//    if(lastCrc != CRC8((byte*)(&rcv), sizeof(rcv))) {
			//      #if SERIAL_DEBUG
			//        Serial.println("Packet rejected: Wrong CRC.");
			//      #endif
			//  
			//      return; //Reject Command. 
			//    }

			//    #if SERIAL_DEBUG
			//    Serial.println("Packet good");
			//    int i;
			//    for (i = 0; i < inputSize; ++i) {
			//      Serial.print(input[i]);
			//      Serial.print(",");
			//    }
			//    #endif
			
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
		DEBUGln(F("[BLE] Timeout Disconnected. [ERR]"));
		isConnected = false;
	}
	return found;
}

void BleHM10Driver::printHex(char X) {

   if (X < 16) {Serial->print("0");}

   Serial->print(X, HEX);

}

void BleHM10Driver::sendObject(void* packet, unsigned int objSize) {
	#if SERIAL_DEBUG_NET
	DEBUGNET(F("[INFO] [BLE] PKT ack: "));
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
		DEBUGln(F("[BLE] Module name not found! [ERR]"));
		return 0;
	}
	
	//message is ok. Read name now
	if (ostream.available()){

		char name[13];
		int bytes = ostream.readBytes(name, 12); //12 is max length by HM-10 doc
		if(bytes == 0) return 0;
		
		name[bytes+1] = '\0'; //String terminator
		
		memcpy(ptr, name, objSize); //Copy the bytes into the struct

		return bytes+1;
	}

	return 0;
}


//CRC-8 - based on the CRC8 formulas by Dallas/Maxim
//code released under the therms of the GNU GPL 3.0 license
byte BleHM10Driver::CRC8(const byte *data, byte len) {
  byte crc = 0x00;
  while (len--) {
    byte extract = *data++;
    for (byte tempI = 8; tempI; tempI--) {
      byte sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}

#endif