#ifndef THREEDRACERS_C
#define THREEDRACERS_C

////////
//
//Written by Marco D'Alia and Davide Marcoccio
//2015
//
//
////////

#include "3DRacers.h"
#include "Config.h"
#include "BleHM10Driver.h"

//------------------------
#include <Arduino.h>
#include <stdlib.h>

#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROMAnything.h>
#include <SoftPWM.h>

ThreeDRacers::ThreeDRacers() :
	//Static fields
	EEPROM_SERVO_CENTER(8),
	EEPROM_SERVO_MAX_ANGLE(16),
	EEPROM_CAR_CALIBRATED(24),
	EEPROM_INVERT_STEERING(32),
	EEPROM_INVERT_THROTTLE(40),
	connectionLedLoop(0),
	gatePower(60),
	gateMinDelay(1 * 1000),//10s
	//non-static fields
	packetCount(0),
	lastPacketCount(0),
	netDebug(false),
	
	//pins
	servoPin(SERVO_PIN),
	sensorPin(SENSOR_PIN),
	
	ledPin(LED_PIN),
	ledStrip(1, LED_PIN, NEO_GRB + NEO_KHZ800),

	BleSerial(NULL),
	Serial(NULL),

	writeRateEEPROM(200),
	nextWriteEEPROM(0),

	nextConnectionCheck(0),
	wasConnected(false),
	
	input(),
	wireless()

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

	#if SHELL_ENABLED || SERIAL_DEBUG || SERIAL_DEBUG_NET
		while (!Serial) {
			; // wait for serial port to connect. Needed for Leonardo only
		}
		delay(5000); //Give time to open the serial console
		//430bytes:
		#if !SERIAL_DEBUG_NET
		STARTLOGFln("  ____  _____  _____");       
		STARTLOGFln(" |___ \\|  __ \\|  __ \\");
		STARTLOGFln("   __) | |  | | |__) |__ _  ___ ___ _ __ ___");
		STARTLOGFln("  |__ <| |  | |  _  // _` |/ __/ _ \\ '__/ __|");
		STARTLOGFln("  ___) | |__| | | \\ \\ (_| | (_|  __/ |  \\__ \\");
		STARTLOGFln(" |____/|_____/|_|  \\_\\__,_|\\___\\___|_|  |___/");
		STARTLOGF  ("               RacerOS v");
		STARTLOGln(VERSION);
		STARTLOGFln("");
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
	
	//Chainable RGB Led
	STARTLOGF("[CAR] RGB Led setup");	
	ledStrip.begin();
	LedColor(255, 255, 0);
	STARTLOGFln(" [OK]");
	
	//Init BLE radio:
	wireless.begin(*BleSerial, *Serial, BLE_RESET);
	nextConnectionCheck = millis();
	
	//Motor init:
	STARTLOGF("[CAR] DC Motor1 setup");	
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
	STARTLOGF("[CAR] Load config from EEPROM");
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
	STARTLOGF(" using Interrupt: ");	
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
	SHELLOUTFln("[CAR] Ready for connection or console commands (eg: /help) [OK]");
	SHELLOUTFln("");
	SHELLOUTFln(":>");
	#endif
}

void ThreeDRacers::Process()
{

	//Started the update routine of the BLE module, 
	//just act as a Serial proxy and be silent to not corrupt data:
	if(wireless.isUpdating) {
	  if (BleSerial->available())
		Serial->write(BleSerial->read());
	  if (Serial->available())
		BleSerial->write(Serial->read());
		
	  return; //Quit to not corrupt data
	}

	//Read incoming data (if any):
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
					SHELLOUTF("[CAR] PKT <- Drive Command (1 of 20): ");
					SHELLOUT(driveCmd.packetCount);
					SHELLOUTF(" brake: ");
					SHELLOUT(driveCmd.brake);
					SHELLOUTF(" reverse: ");
					SHELLOUT(driveCmd.reverse);
					SHELLOUTF(" steerAngle: ");
					SHELLOUT(driveCmd.steerAngle);
					SHELLOUTF(" throttle: ");
					SHELLOUTln(driveCmd.throttle);
				}
				#endif
				processDriveCommand(driveCmd, carInfo);

				//Send Ack (every 3 pkt):
				if(driveCmd.packetCount % 3 == 0 || configCmd.flags.gateSensorDebug) {
					OnAckNotification();
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
	
				if(!configCmd.flags.enableGateSensor) {
					pinMode(SENSOR_LED_PIN, OUTPUT);
					digitalWrite(SENSOR_LED_PIN, LOW);
				}
				else {
					pinMode(SENSOR_LED_PIN, INPUT); //High-z
				}
				
				EEPROM_writeAnything(EEPROM_INVERT_STEERING, configCmd.flags.invertSteering);
				EEPROM_writeAnything(EEPROM_INVERT_THROTTLE, configCmd.flags.invertThrottle);
				
				if(!configCmd.flags.calibrated) {
					configCmd.flags.calibrated = true;
					EEPROM_writeAnything(EEPROM_CAR_CALIBRATED, configCmd.flags.calibrated);
				}

				processConfigCommand(configCmd, carInfo);
			}
			break;
		case NAME_CMD_ID: //change name
			memcpy(&nameCmd, &input[0], sizeof(nameCmd));
			#if SHELL_ENABLED
			if(netDebug) {
				SHELLOUTF("[CAR] PKT <-");				
				SHELLOUTF("nameCmd: ");
				SHELLOUTln(nameCmd.name);
			}
			#endif
			wireless.reset();
			BleSerial->print("AT+NAME");
			BleSerial->print(nameCmd.name);
			wireless.initialize();
			delay(100);
			break;
		default:
			#if SHELL_ENABLED
			if(netDebug) {
				SHELLOUTF("[CAR] PKT <- Unknown Packet: ");
				SHELLOUT(id);
				SHELLOUTFln(" [ERR]");
			}
			#endif
			break;
		}
				
	}
		
	//check connection status
	if (nextConnectionCheck < millis())
	{
		ConnectionStatus();
		ackCmd.batteryLevel = readVcc();
		nextConnectionCheck = millis() + CONNECTION_CHECK_RATE;
	}
	
	//Serial commands:
	#if SHELL_ENABLED
	if (Serial && Serial->available() > 0) {
		String command = Serial->readStringUntil('\n');
		if(command && command.length() > 0) {
			SHELLOUT(":> ");
			SHELLOUTln(command);
			
			if(command.startsWith("/at ")) {
				command.remove(0, 4);
				wireless.sendCommand(command); //todo: find a way to proxy te response back
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
					SHELLOUTF("Wrong angle value: ");
					SHELLOUT(angle);
					SHELLOUTFln(" [ERR]");
				}
				else {
					SetServoCenter(command.toInt());
					SHELLOUTF("Set servo1 center: ");
					SHELLOUT(angle);
					SHELLOUTFln(" [OK]");
				}
			}
			else if(command.startsWith("/servo1 max ")) {
				command.remove(0, 12);
				SHELLOUTln(command);
				int angle = command.toInt();
				if(angle <= 0) {
					SHELLOUTF("Wrong angle value: ");
					SHELLOUT(angle);
					SHELLOUTFln(" [ERR]");
				}
				else {
					SetServoMaxAngle(command.toInt());
					SHELLOUTF("Set servo1 max radius: ");
					SHELLOUT(angle);
					SHELLOUTFln(" [OK]");
				}
			}
			else if(command.startsWith("/netdebug")) {
				netDebug = !netDebug;
				if(netDebug) {
					SHELLOUTFln("Net Packets tracing Enabled [OK]");
				}
				else {
					SHELLOUTFln("Net Packets tracing Disabled [OK]");
				}
			}
			else if(command.startsWith("/help")) {
				helpShellCommand();
			}
			else {
				SHELLOUTFln("Unknown Command [ERR]");
				helpShellCommand();
			}
		}
	}
	#endif
}

//[INFO] [BLE] PKT freq: #          8ms - byte:17 frag:0 hex: 00 00 42 00 00 00 00 00 00 04 22 00 00 00 00 00 00 1057
//66

bool ThreeDRacers::updateDriveCommand(char* input)
{
	if(!packetCheck(input)) {
		return false;
	}
	memcpy(&driveCmd, &input[0], sizeof(driveCmd));
	return true;
}

void ThreeDRacers::OnAckNotification()
{
	ackCmd.packetCount = lastPacketCount;
	ackCmd.sensorLevel = (short) analogRead(sensorPin);
	
	memcpy(&input[0], &ackCmd, sizeof(ackCmd));
	
	#if SHELL_ENABLED
	if(netDebug && ackCmd.packetCount % 20 == 0) {
		SHELLOUTF("[CAR] PKT -> Ack sent back (1 of 20): ");
		SHELLOUT(ackCmd.packetCount);
		SHELLOUTF(" bat: "); SHELLOUT(ackCmd.batteryLevel);
		SHELLOUTF(" sens: "); SHELLOUT(ackCmd.sensorLevel);
		SHELLOUTF(" gate: "); SHELLOUTln(ackCmd.lastGateDetected);
	}
	#endif
	wireless.sendObject(input, sizeof(input));
}

bool ThreeDRacers::updateConfigCommand(char* input)
{
	if(!packetCheck(input)) {
		return false;
	}
	memcpy(&configCmd, &input[0], sizeof(configCmd));
	return true;
}

bool ThreeDRacers::packetCheck(char* input) {
	DriveCommand* cmd = (DriveCommand*) input; //Just cast to one of the struct, the first part is the same (id and packetCount)
	
	//TODO: do a proper check
	if (lastPacketCount == cmd->packetCount) {
		#if SHELL_ENABLED
		if(netDebug) SHELLOUTFln("[CAR] [ERR] Packet rejected: Duplicate packetCount.");
		#endif
		return false; //Reject Command. 
	}
	return true;
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
	ledStrip.setPixelColor(0, ledStrip.Color(red, green, blue));
	ledStrip.show();
}

void ThreeDRacers::OnGateDetected(float gateTime)
{
	float lapTime = gateTime - ackCmd.lastGateDetected;
	ackCmd.lastGateDetected = gateTime;

	//send lap packet
	char id = 0;
	char packet[5];

	unsigned char buf[sizeof(float)];
	memcpy(buf, &lapTime, sizeof(float));

	packet[0] = id;
	packet[1] = buf[0];
	packet[2] = buf[1];
	packet[3] = buf[2];
	packet[4] = buf[3];

	DEBUG(F("lap time in char "));
	for (int i = 0; i < 4; i++)
	{
		DEBUGln((int)buf[i]);
	}
	DEBUGln(F(""));

	DEBUG(F("lap time sent back: "));
	DEBUGln(lapTime);
	wireless.sendObject(&packet, sizeof(packet));
}

int ThreeDRacers::compensateBatteryLoss(int speed)
{
	return constrain(speed * (carInfo.maxBatteryLevel / ackCmd.batteryLevel), 0, 254);
}

void ThreeDRacers::ConnectionStatus() {
	if (wireless.isConnected == false) { //Pulse
		if (connectionLedLoop > 500) {
			connectionLedLoop = 0;
		}

		LedColor(0, 0, abs(connectionLedLoop - 255));
		
		connectionLedLoop += 4;
		
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
			SHELLOUTF("[CAR] PKT ->");
			debugConfigPck();
		}
		#endif
		memcpy(&input[0], &configCmd, sizeof(configCmd));
		wireless.sendObject(input, sizeof(input));
	}
	else {
		MotorStop();
		SetServo(0);
	}
}


void ThreeDRacers::debugConfigPck() {
#if SHELL_ENABLED
	SHELLOUTF(" Car Config Info. steerCenter: ");
	SHELLOUT(configCmd.steerCenter);
	if(configCmd.flags.steerCenterChanged) SHELLOUTF(" X");
	SHELLOUTF(" steerMax: ");
	SHELLOUT(configCmd.steerMax);
	if(configCmd.flags.steerMaxChanged) SHELLOUTF(" X");
	
	SHELLOUTF("invertSteering: ");
	if(configCmd.flags.invertSteering) SHELLOUTF(" X");
	
	SHELLOUTF("invertThrottle: ");
	if(configCmd.flags.invertThrottle) SHELLOUTF(" X");
	
	SHELLOUTF(" sensor: ");
	SHELLOUT(configCmd.flags.enableGateSensor);
	SHELLOUTF(" dbg: ");
	SHELLOUTln(configCmd.flags.gateSensorDebug);
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
	int steerAngle = map(value, invert*-90, invert* 90, configCmd.steerCenter + invert*configCmd.steerMax, configCmd.steerCenter - invert*configCmd.steerMax); //Nb: inverted min and max (inverted rotation since the servo is upsidedown)
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

#if SOFTWARE_PWM
	SoftPWMBegin();
	SoftPWMSetFadeTime(13, 100, 500);
#endif

}

void ThreeDRacers::MotorControl(int controlSpeed, bool brake) {

	controlSpeed = (configCmd.flags.invertThrottle ? -1 : 1) * controlSpeed;
	
#if MOTOR_MODE_PH_EN
	if (brake) { //Brake:
		//pinMode(MOTOR_PIN1, INPUT);
		digitalWrite(MOTOR_PIN1, LOW); //<- TODO: doesn't work, just coasts
		
		analogWrite(MOTOR_PIN2, LOW);
		#if SOFTWARE_PWM
			SoftPWMSet(MOTOR_PIN2, 0);
		#endif
	}
	else {
		//pinMode(MOTOR_PIN1, OUTPUT);

		if (controlSpeed == 0) { //Coasts:
			digitalWrite(MOTOR_PIN1, HIGH);
#if SOFTWARE_PWM
			SoftPWMSet(MOTOR_PIN2, 0);
#else
			analogWrite(MOTOR_PIN2, LOW);
#endif 
		}
		else if (controlSpeed > 0) { //Forward
			digitalWrite(MOTOR_PIN1, HIGH);
#if SOFTWARE_PWM
			SoftPWMSet(MOTOR_PIN2, abs(controlSpeed));
#else
			analogWrite(MOTOR_PIN2, abs(controlSpeed));
#endif 
		}
		else {
			digitalWrite(MOTOR_PIN1, LOW);
#if SOFTWARE_PWM
			SoftPWMSet(MOTOR_PIN2, abs(controlSpeed));
#else
			analogWrite(MOTOR_PIN2, abs(controlSpeed));
#endif 
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
	char name[13];
	int found = wireless.getBLEName(*BleSerial, name, sizeof(name));
	if(found) {
		SHELLOUT(name);
	}
	else {
		SHELLOUTF("BLE-ERROR");
	}

	SHELLOUTF(" - RacersOS v");
	SHELLOUT(VERSION);
	SHELLOUTF(".");
	SHELLOUT(MINOR_VERSION);
	SHELLOUTF(" (p");
	SHELLOUT(PROTOCOL_VERSION);
	SHELLOUTFln(")");
	SHELLOUT("Battery Level: ");
	SHELLOUT(round(100.0 / (1.0*(carInfo.maxBatteryLevel - carInfo.minBatteryLevel) / (ackCmd.batteryLevel - carInfo.minBatteryLevel))));
	SHELLOUTF("% (");
	SHELLOUT(ackCmd.batteryLevel / 1000.0);
	SHELLOUTFln("v)");
	
	SHELLOUTF("Servo config: center: ");
	SHELLOUT(configCmd.steerCenter);
	SHELLOUTF(" max radius: ");
	SHELLOUT(configCmd.steerMax);
	if(configCmd.flags.invertSteering) {
		SHELLOUTF("(inverted)");
	}
	SHELLOUTFln("");
	
	if(configCmd.flags.invertThrottle) {
		SHELLOUTFln("Throttle: inverted)");
	}

	SHELLOUTF("Remote: ");
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
	SHELLOUTFln("/showinfo : Get actual car configuration");
	SHELLOUTFln("/motorscheck : Move the servos R/L and the motors F/B");
	SHELLOUTFln("/servo1 center <angle> : Set servo center position");
	SHELLOUTFln("/servo1 max <angle> : Set servo max rotation angle");
	SHELLOUTFln("/netdebug : Enable/Disable net packets trace");
	SHELLOUTFln("");
	SHELLOUTFln("Type in your console to input command");
	SHELLOUTFln(":>");
	#endif
}

void ThreeDRacers::checkMotorsCommand() {
	#if SHELL_ENABLED
	SHELLOUTF("Test Servo1: [RIGHT]");
	servo.write(driveCmd.steerAngle + configCmd.steerMax);
	delay(500);
	SHELLOUTF(" [LEFT]");
	servo.write(driveCmd.steerAngle - configCmd.steerMax);
	delay(500);
	servo.write(driveCmd.steerAngle);
	SHELLOUTFln(" [COMPLETED]");
	
	delay(500);
	
	SHELLOUTF("Testing MotorA [FORWARD]");
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
	SHELLOUTF(" [BACKWARD]");
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
	SHELLOUTFln(" [COMPLETED]");
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

#endif