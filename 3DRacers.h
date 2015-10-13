////////
//
//Written by Marco D'Alia and Davide Marcoccio
//2015
//
//
////////

#ifndef THREEDRACERS_H
#define THREEDRACERS_H

#include <Arduino.h>

//------------------------

#include <Servo.h>
#include <Adafruit_NeoPixel.h>

#include "3DRacers_Packets.h"
#include "BleHM10Driver.h"

//#include "variants/3DRacers_Pins_1.1.0.h"
//#include "variants/3DRacers_Pins_1.2.0.h"
//#include "variants/3DRacers_Pins_1.4.5.h"
#include "variants/3DRacers_Pins_1.5.6.h"
//#include "variants/3DRacers_Pins_Breadboard.h"
#include "Config.h"

#define VERSION 0
#define MINOR_VERSION 9

#define PROTOCOL_VERSION 1

#define BLE_CMD_SIZE 17
#define CONNECTION_CHECK_RATE 50

typedef struct CarInfo
{
	//car info
	int maxBatteryLevel;
	int minBatteryLevel;

	//drive state
	short throttle;//From 0 to 1024
	short steerAngle;//From -90 to 90
	bool brake;
	bool reverse;

	CarInfo()
	{
		maxBatteryLevel = 4200;
		minBatteryLevel = 2800;
		
		throttle = 0;
		steerAngle = 45;
		brake = false;
		reverse = false;
	}
	
} CarInfo;

class ThreeDRacers
{

public:

	int gateMinDelay; 
	int gatePower;
	
	int servoPin;
	int sensorPin;
	int ledPin;
	
	bool netDebug;
	Servo servo;
	
	Adafruit_NeoPixel ledStrip;
	
	
	ThreeDRacers();
	
	//Use to start in your setup() function
	void Begin(HardwareSerial &bleSerial, Serial_ &serial);
	
	//Call it repeatly in your loop() function:
	void Process();

	//Control methods for your bot:
	void SetServo(int value); //value must be in the [-90, 90] range, will be automatically mapped to the actual steer range (NB: Hardware servo is 90° for center, range from to +-60)
	void SetServoCenter(int value);
	void SetServoMaxAngle(int value);

	void MotorMoveForward(int speed);
	void MotorMoveBackwards(int speed);
	void MotorStop();
	void MotorBrake();
	
	void LedColor(byte red, byte green, byte blue);
	
	//to be used for low level control, use Motor* functions if possible
	void MotorControl(int controlSpeed, bool brake);

	
	//CALLBACKS

	//Set this callbacks to your functions to customize the bot behaviour:
	void OnConnect(void(*f)(CarInfo&, ConfigCommand&)){ processConnectCommand = f; }	
	void OnDriveCommand(void(*f)(DriveCommand&, CarInfo&)){ processDriveCommand = f; }		
	void OnConfigCommand(void(*f)(ConfigCommand&, CarInfo&)){ processConfigCommand = f; }
	void OnRaw1Command(void(*f)(void*, CarInfo&)){ processRaw1Command = f; }
//	void (*OnGateDetected)(CarInfo& car){}

	//TX towards central device
	void OnAckNotification();


private:
	int connectionLedLoop;
	bool wasConnected;
	
	const int EEPROM_SERVO_CENTER;
	const int EEPROM_SERVO_MAX_ANGLE;
	const int EEPROM_CAR_CALIBRATED;
	const int EEPROM_INVERT_STEERING;
	const int EEPROM_INVERT_THROTTLE;
	
	char input[BLE_CMD_SIZE];
	unsigned long packetCount;
	unsigned long lastPacketCount;
	unsigned long nextConnectionCheck;
	BleHM10Driver wireless;

	CarInfo carInfo;
	DriveCommand driveCmd;
	ConfigCommand configCmd;
	AckCommand ackCmd;
	NameCommand nameCmd;
	
	//
	/*------------------------------------------------------*/
	//
	
	unsigned long writeRateEEPROM;
	unsigned long nextWriteEEPROM;

	Serial_* Serial;
	HardwareSerial* BleSerial;

	//
	/*------------------------------------------------------*/
	//

	//function ptrs to UPCs(user provided callbacks)
	void(*processConnectCommand)(CarInfo& car, ConfigCommand& config);
	void(*processDriveCommand)(DriveCommand& cmd, CarInfo& car);
	void(*processConfigCommand)(ConfigCommand& cmd, CarInfo& car);
	void(*processRaw1Command)(void* cmd, CarInfo& car);

	//called inside Begin()
	void motorSetup();
	//unify 2 bytes into an int16
	short bitShiftCombine(unsigned char x_high, unsigned char x_low);
	int compensateBatteryLoss(int speed);

	void OnGateDetected(float gateTime);

	void ConnectionStatus();
	void connectionChanged(bool connected);
	void debugConfigPck();
	bool packetCheck(char* input);
	
	//structs update funcs, called right before it's function ptr companion
	bool updateDriveCommand(char* input);
	bool updateConfigCommand(char* input);

	void helpShellCommand();
	void showInfoShellCommand();
	void checkMotorsCommand();
	
	long readVcc();
};

#endif