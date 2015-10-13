
#define DRIVE_CMD_ID 0
#define CONFIG_CMD_ID 1
#define ACK_CMD_ID 2
#define NAME_CMD_ID 3

//NB: avr-gcc on Atmega32u4 seems to use already a 1byte alignment for structs, however to be sure we'll set the packed attribute here.

//15bytes
struct __attribute__ ((packed)) DriveCommand 
{
	char id;
	unsigned long packetCount;
	
	short throttle;//From 0 to 1024 //2B
	short steerAngle;//From -90 to 90 //2B
	bool brake; 
	bool reverse;
	
	byte red;
	byte green;
	byte blue;
	bool changeColor;
	
	DriveCommand()
	{
		id = DRIVE_CMD_ID;
		packetCount = 0;
		throttle = 0;
		steerAngle = 45;
		brake = false;
		reverse = false;
		red = 0;
		green = 0;
		blue = 0;
		changeColor = false;
	}
	
};
typedef struct DriveCommand DriveCommand;

//13bytes
struct __attribute__ ((packed)) AckCommand
{
	char id;
	unsigned long packetCount;
	
	short batteryLevel;
	short sensorLevel;
	unsigned long lastGateDetected; //NB: it seems that there are some problems with unsigned longs in c#
	
	AckCommand()
	{
		id = ACK_CMD_ID;
		packetCount = 0;
		batteryLevel = 0;
		sensorLevel = 0;
		lastGateDetected = 0;
	}
	
};
typedef struct AckCommand AckCommand;

typedef byte boolean_t;
typedef union {
        struct {
                boolean_t enableGateSensor:1;
                boolean_t gateSensorDebug:1;
                boolean_t calibrated:1;
				boolean_t steerCenterChanged:1;			
                boolean_t steerMaxChanged:1;     
				
				boolean_t invertSteering:1;     
				boolean_t invertThrottle:1;
				boolean_t eighth:1;     				
        };
        byte raw;
} flags_cfg;

//10bytes
struct __attribute__ ((packed)) ConfigCommand
{

	char id;
	unsigned long packetCount;
	
	byte version;
	byte minorVersion;
	byte protocolVersion;
	
	flags_cfg flags;
	
	short steerCenter; //2B
	short steerMax; //2B

	
	ConfigCommand()
	{
		id = CONFIG_CMD_ID;
		packetCount = 0;
		
		steerCenter = 90;
		steerMax = 10;
		flags.steerCenterChanged = false;
		flags.steerMaxChanged = false;
		version = 0;
		minorVersion = 0;
		protocolVersion = 0;
		flags.enableGateSensor = false;
		flags.gateSensorDebug = false;
		flags.calibrated = false;
		flags.invertSteering = false;
		flags.invertThrottle = false;
	}

};
typedef struct ConfigCommand ConfigCommand;

struct __attribute__ ((packed)) NameCommand
{

	char id;
	unsigned long packetCount;
	
	char name[12]; 
	
	NameCommand()
	{
		id = NAME_CMD_ID;
		packetCount = 0;
	}

};
typedef struct NameCommand NameCommand;