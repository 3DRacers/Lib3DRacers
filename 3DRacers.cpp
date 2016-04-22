#ifndef THREEDRACERS_C
#define THREEDRACERS_C

#include "3DRacers.h"
#include "Config.h"
#include "BleHM10Driver.h"
#include "IRSensor.h"

//------------------------
#include <Arduino.h>
#include <stdlib.h>

#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROMAnything.h>

#include <sha204_library.h>
#include <sha204_includes/sha204_lib_return_codes.h>

ThreeDRacers::ThreeDRacers() :
	//Static fields
	EEPROM_SERVO_CENTER(8),
	EEPROM_SERVO_MAX_ANGLE(16),
	EEPROM_CAR_CALIBRATED(24),
	EEPROM_INVERT_STEERING(32),
	EEPROM_INVERT_THROTTLE(40),
	connectionLedLoop(0),
	//non-static fields
	packetCount(0),
	lastPacketCount(0),
	netDebug(false),
	
	//pins
	servoPin(SERVO_PIN),
	sensorPin(SENSOR_PIN),
	
	identity(IDENTITY_PIN),
	
	tickCount(0),
	
	ledPin(LED_PIN),
	#if !REDUCED_FEATURES
	ledStrip(1, LED_PIN, NEO_GRB + NEO_KHZ800),
	#endif
	
	BleSerial(NULL),
	Serial(NULL),

	writeRateEEPROM(200),
	nextWriteEEPROM(0),

	nextConnectionCheck(0),
	wasConnected(false),
	
	input(),
	wireless(),
	sensor(),
	
	processConnectCommand(NULL),
	processDriveCommand(NULL),
	processConfigCommand(NULL),
	processOnGateDetected(NULL),
	processRaw1Command(NULL)

{
}

//<summary>
//Streams instances must be initialized before calling this functions
//</sumary>
void ThreeDRacers::Begin(HardwareSerial &bleSerial, Serial_ &serial)
{ 
	BleSerial = &bleSerial;
	Serial = &serial;
	
	configCmd.version = VERSION;
	configCmd.minorVersion = MINOR_VERSION;
	configCmd.protocolVersion = PROTOCOL_VERSION;
				
	//Chainable led:
	#if !REDUCED_FEATURES
	ledStrip.begin();
	LedColor(255, 255, 0);
	#endif
	
	#if SHELL_ENABLED && STARTUP_LOG
	//Place a finger on the sensor to wait for the startup sequence to start (giving time to connecto to the Arduino console)
	pinMode(SENSOR_LED_PIN, OUTPUT);
	digitalWrite(SENSOR_LED_PIN, LOW);
	delay(100);
	if(analogRead(SENSOR_PIN) < 600) {
		delay(10000); //Give time to open the serial console
	}
	#endif
	
	#if SHELL_ENABLED || SERIAL_DEBUG || SERIAL_DEBUG_NET
		while (!Serial) {
			; // wait for serial port to connect. Needed for Leonardo only
		}
		
		//430bytes:
		#if !SERIAL_DEBUG_NET && STARTUP_LOG
		STARTLOGFln("  ____  _____  _____");       
		STARTLOGFln(" |___ \\|  __ \\|  __ \\");
		STARTLOGFln("   __) | |  | | |__) |__ _  ___ ___ _ __ ___");
		STARTLOGFln("  |__ <| |  | |  _  // _` |/ __/ _ \\ '__/ __|");
		STARTLOGFln("  ___) | |__| | | \\ \\ (_| | (_|  __/ |  \\__ \\");
		STARTLOGFln(" |____/|_____/|_|  \\_\\__,_|\\___\\___|_|  |___/");
		STARTLOGF  ("            RacerOS v");
		STARTLOG(VERSION);
		STARTLOGF(".");
		STARTLOG(MINOR_VERSION);
		STARTLOGF(" (net: ");
		STARTLOG(PROTOCOL_VERSION);
		STARTLOGFln(")");
		STARTLOGFln("Initializing...");
		#endif
	#endif

	//Init pins:
	STARTLOGF("[ARDU] Init Pin Modes");
	pinMode(SENSOR_PIN, INPUT);
	pinMode(SENSOR_LED_PIN, INPUT); //High-z
	pinMode(BLE_RESET, INPUT);
	pinMode(SERVO_PIN, OUTPUT);
	pinMode(LED_PIN, OUTPUT);
	pinMode(MOTOR_MODE_PIN, OUTPUT);
	pinMode(MOTOR_PIN1, OUTPUT);
	pinMode(MOTOR_PIN2, OUTPUT);
	delay(10);
	STARTLOGFln(" [OK]");
	
	//Init BLE radio:
	wireless.begin(*BleSerial, *Serial, BLE_RESET);
	if(wireless.isUpdating) {
		//Go in BLE update mode and interrupt the init process:
		LedColor(255, 0, 255);
		return;
	}
	
	nextConnectionCheck = millis();
	
	//Motor init:
	STARTLOGF("[CAR] DC MotorA setup");	
	motorSetup();

	//Welcome sequence:
	#if MOTORS_DEBUG
	MotorControl(-150, false);
	delay(200);
	MotorControl(150, false);
	delay(200);
	MotorControl(0, false);
	#endif
	STARTLOGFln(" [OK]");
	
	//Servo init:
	STARTLOGF("[CAR] Load EEPROM");
	if (EEPROM.read(EEPROM_SERVO_CENTER) != 255) {
		EEPROM_readAnything(EEPROM_SERVO_CENTER, configCmd.steerCenter);
	}
	if (EEPROM.read(EEPROM_SERVO_MAX_ANGLE) != 255) {
		EEPROM_readAnything(EEPROM_SERVO_MAX_ANGLE, configCmd.steerMax);
	}
	if (EEPROM.read(EEPROM_CAR_CALIBRATED) != 255) {
		bool calibrated = false;
		EEPROM_readAnything(EEPROM_CAR_CALIBRATED, calibrated);
		configCmd.flags.calibrated = calibrated;
	}
	if (EEPROM.read(EEPROM_INVERT_STEERING) != 255) {
		bool invertSteering = false;
		EEPROM_readAnything(EEPROM_INVERT_STEERING, invertSteering);
		configCmd.flags.invertSteering = invertSteering;
	}
	if (EEPROM.read(EEPROM_INVERT_THROTTLE) != 255) {
		bool invertThrottle = false;
		EEPROM_readAnything(EEPROM_INVERT_THROTTLE, invertThrottle);
		configCmd.flags.invertThrottle = invertThrottle;
	}
	STARTLOGFln(" [OK]");	
	
	//Attach Servo
	STARTLOGF("[CAR] Servo setup");	
	STARTLOGF(" int.: ");	
	#ifdef _useTimer1
		STARTLOGF("1 ");
	#endif
	#ifdef _useTimer2
		STARTLOGF("2 ");
	#endif
	#ifdef _useTimer3
		STARTLOGF("3 ");
	#endif
	#ifdef _useTimer4
		STARTLOGF("4 ");
	#endif
	#ifdef _useTimer5
		STARTLOGF("5 S");
	#endif
	
	driveCmd.steerAngle = configCmd.steerCenter;
	servo.attach(servoPin);

	//Welcome sequence:
	#if MOTORS_DEBUG
	servo.write(driveCmd.steerAngle + configCmd.steerMax);
	delay(500);
	servo.write(driveCmd.steerAngle - configCmd.steerMax);
	delay(500);
	servo.write(driveCmd.steerAngle);
	#endif
	STARTLOGFln(" [OK]");
	
	#if SHELL_ENABLED
	SHELLOUTFln("[CAR] Ready (eg: /help) [OK]");
	SHELLOUTFln("");
	SHELLOUTFln(":>");
	#endif
}

void ThreeDRacers::Process()
{
	//Started the update routine of the BLE module, 
	//just act as a Serial proxy and be silent to not corrupt data:
	if(wireless.isUpdating) {
	  if (Serial->available())
		BleSerial->write(Serial->read());
	  if (BleSerial->available())
		Serial->write(BleSerial->read());
		
	  return; //Quit to not corrupt data
	}
	
	tickCount++;
	
	//process IR reflective sensor gates detection:
	ackCmd.sensorLevel = (short) analogRead(sensorPin);
	if(sensor.process(ackCmd.sensorLevel)) {
		SHELLOUTFln("[CAR] Gate detected ");				
		ackCmd.lastGateDetected = sensor.lastSequenceStart;
		ackCmd.lastGateDuration = sensor.lastSequenceDuration;
		if(processOnGateDetected != NULL) processOnGateDetected(ackCmd, carInfo);
		SendAckNotification();
	}
	else {
		ackCmd.lastGateDetected = 0;
		ackCmd.lastGateDuration = 0;
	}
	
	//Process the other parts only 1 out of 10 times:
	if(tickCount % 10 != 0) {
		return;
	}

	//Read incoming data (if any):
	processPackets();
		
	//check connection status
	if (nextConnectionCheck < millis())
	{
		ConnectionStatus();
		ackCmd.batteryLevel = readVcc();
		nextConnectionCheck = millis() + CONNECTION_CHECK_RATE;
	}

	//Serial commands:
	#if SHELL_ENABLED
		processSerial();
	#endif
}

void ThreeDRacers::processPackets() {
	lastPacketCount = packetCount;
	if (wireless.receive(input, sizeof(input))) {

		//parse byte[] to rcv struct, byte 0 = crc
		//commandType, byte 0 to int
		int id = (int)input[0];

		switch (id)
		{
		case DRIVE_CMD_ID: //drive command
			if (updateDriveCommand(input))
			{
				packetCount = driveCmd.packetCount;
				#if SHELL_ENABLED
				if(netDebug && driveCmd.packetCount % 20 == 0) {
					SHELLOUTF("[CAR] PKT <- Drive (1 of 20): ");
					SHELLOUT(driveCmd.packetCount);
					SHELLOUTF(" brake: ");
					SHELLOUT(driveCmd.brake);
					SHELLOUTF(" rev: ");
					SHELLOUT(driveCmd.reverse);
					SHELLOUTF(" angle: ");
					SHELLOUT(driveCmd.steerAngle);
					SHELLOUTF(" thr: ");
					SHELLOUTln(driveCmd.throttle);
				}
				#endif
				if(processDriveCommand != NULL) processDriveCommand(driveCmd, carInfo);

				//Send Ack (every 3 pkt):
				if((driveCmd.packetCount % 3 == 0 || configCmd.flags.gateSensorDebug ) /* don't double send if in this tick we found a gate */ && ackCmd.lastGateDetected == 0) {
					SendAckNotification();
				}
			}
			break;
		case CONFIG_CMD_ID: //config command, set Servo, motor etc configs

			if (updateConfigCommand(input))
			{
				packetCount = configCmd.packetCount;
				#if SHELL_ENABLED
				if(netDebug) {
					SHELLOUTF("[CAR] PKT <-");				
					debugConfigPck();
				}
				#endif
	
				if(configCmd.flags.enableGateSensor) {
					pinMode(SENSOR_LED_PIN, OUTPUT);
					digitalWrite(SENSOR_LED_PIN, LOW);
				}
				else {
					pinMode(SENSOR_LED_PIN, INPUT); //High-z
				}
				sensor.threshold = (int) configCmd.sensorThreshold;
				
				EEPROM_writeAnything(EEPROM_INVERT_STEERING, configCmd.flags.invertSteering);
				EEPROM_writeAnything(EEPROM_INVERT_THROTTLE, configCmd.flags.invertThrottle);
				
				if(!configCmd.flags.calibrated) {
					configCmd.flags.calibrated = true;
					EEPROM_writeAnything(EEPROM_CAR_CALIBRATED, configCmd.flags.calibrated);
				}

				if(processConfigCommand != NULL) processConfigCommand(configCmd, carInfo);
			}
			break;
		#if !REDUCED_FEATURES
		case NAME_CMD_ID: //change name
			memcpy(&nameCmd, &input[0], sizeof(nameCmd));
			#if SHELL_ENABLED
			if(netDebug) {
				SHELLOUTF("[CAR] PKT <-");				
				SHELLOUTF("n: ");
				SHELLOUTln(nameCmd.name);
			}
			#endif

			wireless.sendCommand(String("AT+NAME") + nameCmd.name);
			wireless.reset();
			wireless.initialize();
			delay(100);
			break;
		#endif
		case IDENTITY_CMD_ID: //ID hash request (the request is splitted in 3 packets)
			memcpy(&idCmd, &input[0], sizeof(idCmd));

			if(idCmd.msgPart == IDENTITY_GET_SIGNATURE_MSG_PART) {

				GetId((uint8_t*) idCmd.payload);
				Send(&idCmd, sizeof(idCmd) - (sizeof(idCmd.payload) - 9) ); //This Cmd has variable size (the payload here is 9 bytes)
				
			}
			else {
				memcpy(idCurrentNonce + (idCmd.msgPart * 12), idCmd.payload, idCmd.msgPart == 2 ? 8 : 12);	
				if(idCmd.msgPart == 2) {
					calculateSignature(idCurrentNonce, idCurrentNonce);
					
					//Send 32byte reply in 3 packets:
					SendIdResponse(0, idCurrentNonce);
					delay(100);
					SendIdResponse(1, idCurrentNonce);
					delay(100);
					SendIdResponse(2, idCurrentNonce);
					delay(100);
				}
			}
			break;
		
		default:
			#if SHELL_ENABLED
			if(netDebug) {
				SHELLOUTF("[CAR] PKT <- Unknown: ");
				SHELLOUT(id);
				SHELLOUTFln(" [ERR]");
			}
			#endif
			break;
		}
				
	}
}

void ThreeDRacers::processSerial() {
	#if SHELL_ENABLED
	if (Serial && Serial->available() > 0) {
		String command = Serial->readStringUntil('\n');
		if(command && command.length() > 0) {
			SHELLOUT(":> ");
			SHELLOUTln(command);
			
			if(command.startsWith("/at ")) {
				command.remove(0, 4);
				wireless.sendCommand(command); //todo: find a way to proxy the response back
			}
			else if(command.startsWith("/showinfo")) {
				showInfoShellCommand();
			}
			else if(command.startsWith("/motorscheck")) {
				checkMotorsCommand();
			}
			else if(command.startsWith("/servo1 center ")) {
				command.remove(0, 15);
				SHELLOUTln(command);
				int angle = command.toInt();
				if(angle <= 0) {
					SHELLOUTF("Wrong: ");
					SHELLOUT(angle);
					SHELLOUTFln(" [ERR]");
				}
				else {
					SetServoCenter(command.toInt());
					SHELLOUTF("Center: ");
					SHELLOUTln(angle);
				}
			}
			else if(command.startsWith("/servo1 max ")) {
				command.remove(0, 12);
				SHELLOUTln(command);
				int angle = command.toInt();
				if(angle <= 0) {
					SHELLOUTF("Wrong: ");
					SHELLOUT(angle);
					SHELLOUTFln(" [ERR]");
				}
				else {
					SetServoMaxAngle(command.toInt());
					SHELLOUTF("Max radius: ");
					SHELLOUTln(angle);
				}
			}
			/*else if(command.startsWith("/pinHigh")) {
				setPin(command, HIGH);
			}
		    else if(command.startsWith("/pinLow ")) {
				setPin(command, LOW);
			}*/
			else if(command.startsWith("/netdebug")) {
				netDebug = !netDebug;
				if(netDebug) {
					SHELLOUTFln("ON");					
				}
				else {
					SHELLOUTFln("OFF");
				}
			}
			else if(command.startsWith("/help")) {
				helpShellCommand();
			}
			else {
				SHELLOUTFln("Unknown [ERR]");
				helpShellCommand();
			}
		}
	}
	#endif
}

/*bool ThreeDRacers::setPin(String &command, bool value) {
	int pin = command.substring(9).toInt();
	digitalWrite(pin, HIGH);
	SHELLOUT(pin);
	if(value) {
		SHELLOUTFln(" HIGH");
	}
	else {
		SHELLOUTFln(" LOW");
	}
} */

//[INFO] [BLE] PKT freq: #          8ms - byte:17 frag:0 hex: 00 00 42 00 00 00 00 00 00 04 22 00 00 00 00 00 00 1057
//66

bool ThreeDRacers::updateDriveCommand(char* input)
{
	if(!packetCheck(input, sizeof(driveCmd))) {
		return false;
	}
	memcpy(&driveCmd, &input[0], sizeof(driveCmd));

	return true;
}

void ThreeDRacers::SendAckNotification()
{
	ackCmd.packetCount = lastPacketCount;

	#if SHELL_ENABLED
	if(netDebug && (ackCmd.packetCount % 20 == 0 || ackCmd.lastGateDetected != 0)) {
		SHELLOUTF("[CAR] PKT -> Ack (1 of 20): ");
		SHELLOUT(ackCmd.packetCount);
		SHELLOUTF(" bat: "); SHELLOUT(ackCmd.batteryLevel);
		SHELLOUTF(" sens: "); SHELLOUT(ackCmd.sensorLevel);
		SHELLOUTF(" gate: "); SHELLOUTln(ackCmd.lastGateDetected);
	}
	#endif
	
	Send(&ackCmd, sizeof(ackCmd));
}

void ThreeDRacers::SendIdResponse(short msgPart, uint8_t* temp_message)
{
	idCmd.packetCount = lastPacketCount;
	idCmd.msgPart = msgPart;
	
	short size = (msgPart == 2) ? 8 : 12;
	memcpy(&idCmd.payload[0], temp_message + (msgPart*12), size);

	#if SHELL_ENABLED
	if(netDebug) {
		SHELLOUTF("[CAR] PKT -> ID HMAC (");
		SHELLOUT(msgPart+1);
		SHELLOUT(" of 3): ");
		
		for (int i=0; i<( msgPart == 2 ? 8 : 12); i++)
		{
		  SHELLOUT(idCmd.payload[i], HEX);
		  SHELLOUTF(" ");
		}	
		SHELLOUTFln("");
		
	}
	#endif
	
	Send(&idCmd, sizeof(idCmd) - (sizeof(idCmd.payload) - size));
}

bool ThreeDRacers::updateConfigCommand(char* input)
{
	if(!packetCheck(input, sizeof(configCmd))) {
		return false;
	}
	memcpy(&configCmd, &input[0], sizeof(configCmd));
	return true;
}

bool ThreeDRacers::packetCheck(char* input, unsigned int objSize) {
	DriveCommand* cmd = (DriveCommand*) input; //Just cast to one of the struct, the first part is the same (id and packetCount)

	if (lastPacketCount == cmd->packetCount) {
		#if SHELL_ENABLED
		if(netDebug) SHELLOUTFln("[CAR] [ERR] Dup. pkt");
		#endif
		return false; //Reject Command. 
	}
	
	byte crc = cmd->crc;
	
	//Calculate CRC8:
	cmd->crc = 0x00;
	byte crcCalc = CRC8((byte*) input, objSize);
	
	if(crc != crcCalc) {
		#if SHELL_ENABLED
		if(netDebug) {
			SHELLOUTF("[CAR] [ERR] Crc:");
			SHELLOUT(crc);
			SHELLOUTF(" != ");
			SHELLOUT(crcCalc);
			SHELLOUTF(" ");
			for(int c=0; c < objSize; c++) {
				if(input[c] < 16) SHELLOUT("0");
				SHELLOUT(input[c], HEX);
				SHELLOUTF(" ");
			}
			SHELLOUTFln("");
		}
		#endif
		//return false;
	}
	return true;
}

void ThreeDRacers::Send(void* data, unsigned int objSize) {
	DriveCommand* cmd = (DriveCommand*) data; //Just cast to one of the struct, the first part is the same (id and packetCount)
	
	//Calculate CRC8:
	cmd->crc = 0x00;
	cmd->crc = CRC8((byte*) data, objSize);
	
	/*
	#if SHELL_ENABLED
		byte* arr = (byte*) data;
		SHELLOUTF("[CAR] [INFO] Crc:");
		SHELLOUT(cmd->crc);
		SHELLOUTF(" ");
		for(int c=0; c < objSize; c++) {
			if(arr[c] < 16) SHELLOUT("0");
			SHELLOUT(arr[c], HEX);
			SHELLOUTF(" ");
		}
		SHELLOUTFln("");
	#endif
	*/
	
	memcpy(&input[0], data, objSize);

	wireless.sendObject(input, BLE_CMD_SIZE);
}

void ThreeDRacers::MotorMoveForward(int speed)
{
	int motorOutput = compensateBatteryLoss(speed);
	MotorControl(motorOutput, false);
}

void ThreeDRacers::MotorMoveBackwards(int speed)
{
	int motorOutput = compensateBatteryLoss(speed);

	MotorControl(-motorOutput, false);
}

void ThreeDRacers::MotorStop()
{
	MotorControl(0, false);
}

void ThreeDRacers::MotorBrake()
{
	MotorControl(0, true);
}

void ThreeDRacers::LedColor(byte red, byte green, byte blue) {
	#if !REDUCED_FEATURES
	ledStrip.setPixelColor(0, ledStrip.Color(red, green, blue));
	ledStrip.show();
	#endif
}

int ThreeDRacers::compensateBatteryLoss(int speed)
{
	return speed;
	//return constrain(speed * (carInfo.maxBatteryLevel / ackCmd.batteryLevel), 0, 254);
}

void ThreeDRacers::ConnectionStatus() {
	if (wireless.isConnected == false) { //Pulse
		if (connectionLedLoop > 400) {
			connectionLedLoop = 0;
		}

		LedColor(0, 0, abs(connectionLedLoop - 255));
		
		connectionLedLoop += 20;
		if(wasConnected) {
			connectionChanged(false);
			wasConnected = false;
		}
	}
	else {
		if (!wasConnected) { //Shut down (only the first time)
			connectionChanged(true);
			LedColor(0, 0, 0);
			wasConnected = true;
		}
	}
}

void ThreeDRacers::connectionChanged(bool connected)
{
	if(connected) {
		configCmd.packetCount = lastPacketCount++;
		#if SHELL_ENABLED
		if(netDebug) {
			//NB: during configuration the cars automaticly disconnect due to a timeout
			SHELLOUTFln("[CAR] Connected [OK]");
			SHELLOUTF("[CAR] PKT ->");
			debugConfigPck();
		}
		#endif

		Send(&configCmd, sizeof(configCmd));
	}
	else {
		#if SHELL_ENABLED
		if(netDebug) {
			SHELLOUTFln("[CAR] Disconnected  [OK]");
		}
		#endif
		MotorStop();
		SetServo(0);
	}
}

void ThreeDRacers::debugConfigPck() {
#if SHELL_ENABLED
	SHELLOUTF(" Car Config Info. sC: ");
	SHELLOUT(configCmd.steerCenter);
	if(configCmd.flags.steerCenterChanged) SHELLOUTF(" X");
	SHELLOUTF(" sM: ");
	SHELLOUT(configCmd.steerMax);
	if(configCmd.flags.steerMaxChanged) SHELLOUTF(" X");
	
	SHELLOUTF(" c: ");
	if(configCmd.flags.calibrated) SHELLOUTF(" X");
	
	SHELLOUTF("iS: ");
	if(configCmd.flags.invertSteering) SHELLOUTF(" X");
	
	SHELLOUTF("iT: ");
	if(configCmd.flags.invertThrottle) SHELLOUTF(" X");
	
	SHELLOUTF(" s: ");
	if(configCmd.flags.enableGateSensor) SHELLOUTF(" X");
	
	SHELLOUTF(" ");
	SHELLOUT(configCmd.sensorThreshold);
	
	SHELLOUTF(" dbg: ");
	if(configCmd.flags.gateSensorDebug) SHELLOUTF(" X");
	
	SHELLOUTFln("");
#endif
}


//<summary>
//combine 2 bytes into a single short
//</summary>
short ThreeDRacers::bitShiftCombine(unsigned char x_high, unsigned char x_low)
{
	short combined;
	combined = x_high;              //send x_high to rightmost 8 bits
	combined = combined << 8;         //shift x_high over to leftmost 8 bits
	combined |= x_low;                 //logical OR keeps x_high intact in combined and fills in rightmost 8 bits
	return combined;
}

void ThreeDRacers::SetServo(int value){
	short invert = configCmd.flags.invertSteering ? -1 : +1;
	int steerAngle = map(value, invert*-90, invert* 90, configCmd.steerCenter - configCmd.steerMax, configCmd.steerCenter + configCmd.steerMax);
	servo.write(steerAngle);
}
void ThreeDRacers::SetServoCenter(int value)
{
	if (millis() > nextWriteEEPROM)
	{
		configCmd.steerCenter = value;
		servo.write(value);
		EEPROM_writeAnything(EEPROM_SERVO_CENTER, value);
		nextWriteEEPROM = millis() + writeRateEEPROM;
	}

}
void ThreeDRacers::SetServoMaxAngle(int value)
{
	if (millis() > nextWriteEEPROM)
	{
		configCmd.steerMax = value;
		servo.write(configCmd.steerCenter + value);
		delay(1000);
		servo.write(configCmd.steerCenter - value);
		delay(1000);
		servo.write(configCmd.steerCenter);

		EEPROM_writeAnything(EEPROM_SERVO_MAX_ANGLE, value);
		nextWriteEEPROM = millis() + writeRateEEPROM;
	}
}

void ThreeDRacers::motorSetup() {
	pinMode(MOTOR_PIN1, OUTPUT);
	pinMode(MOTOR_PIN2, OUTPUT);

	//Motor mode:
	pinMode(MOTOR_MODE_PIN, OUTPUT);
	digitalWrite(MOTOR_MODE_PIN, MOTOR_MODE_PH_EN ? HIGH : LOW); //Motor mode: Phase/Enable

}

void ThreeDRacers::MotorControl(int controlSpeed, bool brake) {

	controlSpeed = (configCmd.flags.invertThrottle ? -1 : 1) * controlSpeed;
	
#if MOTOR_MODE_PH_EN
	//PHase/ENable mode:
	if (brake) { //Brake:
		//pinMode(MOTOR_PIN1, INPUT);  //<- TODO: doesn't work
		
		digitalWrite(MOTOR_MODE_PIN, HIGH);
		digitalWrite(MOTOR_PIN1, LOW);
		analogWrite(MOTOR_PIN2, LOW);
	}
	else {
		pinMode(MOTOR_PIN1, OUTPUT);

		if (controlSpeed == 0) { //Coasts (Switching to IN/IN mode since Phase/Enable doesn't support coasting):
			#if MOTOR_RUN_METHOD == 0
				digitalWrite(MOTOR_MODE_PIN, LOW);
			#endif
			digitalWrite(MOTOR_PIN1, LOW);
			analogWrite(MOTOR_PIN2, LOW);
		}
		else {
			digitalWrite(MOTOR_MODE_PIN, HIGH);
			
			if (controlSpeed > 0) { //Forward
			digitalWrite(MOTOR_PIN1, HIGH);
			analogWrite(MOTOR_PIN2, abs(controlSpeed));
			}
			else {
				digitalWrite(MOTOR_PIN1, LOW);
				analogWrite(MOTOR_PIN2, abs(controlSpeed));
			}
		}
	} 
#else
	if (brake) { //Brake:
		digitalWrite(MOTOR_PIN1, HIGH);
		digitalWrite(MOTOR_PIN2, HIGH);
	}
	else if (controlSpeed == 0) { //Coasts:
		digitalWrite(MOTOR_PIN1, LOW);
		digitalWrite(MOTOR_PIN2, LOW);
	}
	if (controlSpeed > 0) { //Forward
		analogWrite(MOTOR_PIN1, MOTOR_RUN_METHOD);
		analogWrite(MOTOR_PIN2, controlSpeed);
	}
	else {
		analogWrite(MOTOR_PIN1, abs(controlSpeed));
		analogWrite(MOTOR_PIN2, MOTOR_RUN_METHOD);
	}
#endif
}

void ThreeDRacers::showInfoShellCommand() {
	#if SHELL_ENABLED
	if(!wireless.isConnected) {
		char name[13];
		int found = wireless.getBLEName(*BleSerial, name, sizeof(name));
		if(found) {
			SHELLOUT(name);
		}
		else {
			SHELLOUTF("BLE-ERROR");
		}
	}

	SHELLOUTF(" - RacersOS v");
	SHELLOUT(VERSION);
	SHELLOUTF(".");
	SHELLOUT(MINOR_VERSION);
	SHELLOUTF(" (p");
	SHELLOUT(PROTOCOL_VERSION);
	SHELLOUTFln(")");
	printId();
	SHELLOUT("Vcc Level: ");
	SHELLOUT(round(100.0 / (1.0*(carInfo.maxBatteryLevel - carInfo.minBatteryLevel) / (ackCmd.batteryLevel - carInfo.minBatteryLevel))));
	SHELLOUTF("% (");
	SHELLOUT(ackCmd.batteryLevel / 1000.0);
	SHELLOUTFln("v)");
	
	SHELLOUTF("Servo: ");
	SHELLOUT(configCmd.steerCenter);
	SHELLOUTF(" max: ");
	SHELLOUT(configCmd.steerMax);
	if(configCmd.flags.invertSteering) {
		SHELLOUTF("(inverted)");
	}
	SHELLOUTFln("");
	
	if(configCmd.flags.invertThrottle) {
		SHELLOUTFln("Throttle: inverted");
	}

	SHELLOUTF("App: ");
	if(wireless.isConnected) {
		SHELLOUTFln("connected");
	}
	else {
		SHELLOUTFln("not connected");
	}
	SHELLOUTFln("");
	SHELLOUTFln(":>");
	#endif
}

void ThreeDRacers::helpShellCommand() {
	#if SHELL_ENABLED
	SHELLOUTFln("/at AT+* : Send BLE commands");
	SHELLOUTFln("/showinfo : Info");
	SHELLOUTFln("/motorscheck : Move motors/servos");
	SHELLOUTFln("/servo1 center <angle> : Set servo center");
	SHELLOUTFln("/servo1 max <angle> : Set servo max angle");
	SHELLOUTFln("/netdebug : Enable/Disable net dbg");
	//SHELLOUTFln("/pinHigh/Low : digitalWrite");
	SHELLOUTFln("");
	SHELLOUTFln(":>");
	#endif
}

void ThreeDRacers::checkMotorsCommand() {
	#if SHELL_ENABLED
	SHELLOUTF("Test Servo1: [R]");
	servo.write(driveCmd.steerAngle + configCmd.steerMax);
	delay(500);
	SHELLOUTF(" [L]");
	servo.write(driveCmd.steerAngle - configCmd.steerMax);
	delay(500);
	servo.write(driveCmd.steerAngle);
	SHELLOUTFln("");
	
	delay(500);
	
	SHELLOUTF("Testing MotorA [F]");
	MotorControl(255, false);
	delay(200);
	MotorControl(150, false);
	delay(200);
	MotorControl(120, false);
	delay(200);
	MotorControl(100, false);
	delay(200);
	MotorControl(80, false);
	delay(500);
	MotorControl(0, false);
	delay(500);
	SHELLOUTF(" [B]");
	MotorControl(-255, false);
	delay(200);
	MotorControl(-150, false);
	delay(200);
	MotorControl(-120, false);
	delay(200);
	MotorControl(-100, false);
	delay(200);
	MotorControl(-80, false);
	delay(500);
	MotorControl(0, false);
	SHELLOUTFln(" [DONE]");
	#endif
}

long ThreeDRacers::readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

void ThreeDRacers::printId() {
	#if SHELL_ENABLED
	SHELLOUTF("Id: ");
	uint8_t rx_buffer[9];
	GetId(rx_buffer);
	
    for (int i=0; i<9; i++)
    {
	  SHELLOUT(rx_buffer[i], HEX);
	  SHELLOUTF(" ");
    }	
	SHELLOUTFln("");
	#endif
}

void ThreeDRacers::GetId(uint8_t* rx_buffer) {
    byte returnValue;
	returnValue = identity.sha204p_wakeup();
    returnValue = identity.getSerialNumber(rx_buffer);
	identity.sha204p_sleep();
}

#define SHA204_SERIAL_SZ    9
#define SHA_MSG_SIZE                    (64)                   //!< SHA message data size
#define HMAC_MODE_SOURCE_FLAG_MATCH     ((uint8_t) 0x04)       //!< HMAC mode bit  2: match TempKey.SourceFlag

void ThreeDRacers::calculateSignature(uint8_t* temp_message, uint8_t* current_nonce) { 
  //uint8_t temp_message[SHA_MSG_SIZE];
  uint8_t rx_buffer[SHA204_RSP_SIZE_MAX];
  uint8_t tx_buffer[SHA204_CMD_SIZE_MAX];

  /*memset(temp_message, 0, 32);
  memcpy(temp_message, message, msgLength);*/

  //memcpy(current_nonce, nonce, NONCE_NUMIN_SIZE_PASSTHROUGH);
  // We set the part of the 32-byte nonce that does not fit into a message to 0xAA
  //memset(&current_nonce[nonceLength], 0xAA, sizeof(current_nonce)-nonceLength);
   
   identity.sha204p_wakeup();
   
  // Program the data to sign into the ATSHA204
  (void)identity.sha204m_execute(SHA204_WRITE, SHA204_ZONE_DATA | SHA204_ZONE_COUNT_FLAG, 8 << 3, 32, temp_message,    /* Full version of lib requires: */ 0, NULL, 0, NULL, //*/
                  WRITE_COUNT_LONG, tx_buffer, WRITE_RSP_SIZE, rx_buffer);


  // Program the nonce to use for the signature (has to be done just before GENDIG due to chip limitations)
  (void)identity.sha204m_execute(SHA204_NONCE, NONCE_MODE_PASSTHROUGH, 0, NONCE_NUMIN_SIZE_PASSTHROUGH, current_nonce, /* Full version of lib requires: */ 0, NULL, 0, NULL, //*/
                  NONCE_COUNT_LONG, tx_buffer, NONCE_RSP_SIZE_SHORT, rx_buffer);

  // Purge nonce when used
  //memset(current_nonce, 0x00, NONCE_NUMIN_SIZE_PASSTHROUGH);

  // Generate digest of data and nonce
  (void)identity.sha204m_execute(SHA204_GENDIG, GENDIG_ZONE_DATA, 8, 0, NULL,    /* Full version of lib requires: */ 0, NULL, 0, NULL, //*/
                  GENDIG_COUNT_DATA, tx_buffer, GENDIG_RSP_SIZE, rx_buffer);

  // Calculate HMAC of message+nonce digest and secret key
  (void)identity.sha204m_execute(SHA204_HMAC, HMAC_MODE_SOURCE_FLAG_MATCH, 0, 0, NULL,     /* Full version of lib requires: */ 0, NULL, 0, NULL, //*/
                  HMAC_COUNT, tx_buffer, HMAC_RSP_SIZE, rx_buffer);

  // Put device back to sleep
  identity.sha204p_sleep();

  memcpy(temp_message, &rx_buffer[SHA204_BUFFER_POS_DATA], MAC_CHALLENGE_SIZE);

  /*for (int i=0; i<sizeof(rx_buffer); i++)
  {
    SHELLOUT(rx_buffer[i], HEX);
    SHELLOUT(' ');
  }
  SHELLOUTFln("");*/
}

const unsigned char CRC7_POLY = 0x91;
//TO calculate manually: http://tomeko.net/online_tools/crc8.php?lang=en
byte ThreeDRacers::CRC8(const uint8_t* message, int len) //Dallas/Maxim
{
  byte crc = 0x00;
  int c = 0;
  byte extract;
  byte sum;
  while (len--) {
    extract = (byte) message[c];
    ++c;
    for (byte tempI = 8; tempI; tempI--) {
      sum = (crc ^ extract) & 0x01;
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