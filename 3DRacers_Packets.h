
#define DRIVE_CMD_ID 0
#define CONFIG_CMD_ID 1
#define ACK_CMD_ID 2
#define NAME_CMD_ID 3
#define IDENTITY_CMD_ID 4


//NB: avr-gcc on Atmega32u4 seems to use already a 1byte alignment for structs, however to be sure we'll set the ((packed)) attribute on each struct.

//15bytes
struct __attribute__ ((packed)) DriveCommand 
{
	char id;
	unsigned int packetCount;
	byte crc;
	
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
		crc = 0x00;
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

//17bytes
struct __attribute__ ((packed)) AckCommand
{
	char id;
	unsigned int packetCount;
	byte crc;
	
	short batteryLevel;
	short sensorLevel;
	unsigned long lastGateDetected;
	unsigned long lastGateDuration;
	
	AckCommand()
	{
		id = ACK_CMD_ID;
		packetCount = 0;
		crc = 0x00;
		batteryLevel = 0;
		sensorLevel = 0;
		lastGateDetected = 0;
		lastGateDuration = 0;
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

//8bytes
struct __attribute__ ((packed)) ConfigCommand
{

	char id;
	unsigned int packetCount;
	byte crc;
	
	byte version;
	byte minorVersion;
	byte protocolVersion;
	
	flags_cfg flags;
	
	short steerCenter; //2B
	short steerMax; //2B
	short sensorThreshold; //2B
	
	ConfigCommand()
	{
		id = CONFIG_CMD_ID;
		packetCount = 0;
		crc = 0x00;
		
		steerCenter = 90;
		steerMax = 10;
		sensorThreshold = 900;
		
		flags.steerCenterChanged = false;
		flags.steerMaxChanged = false;
		version = 0;
		minorVersion = 0;
		protocolVersion = 0;
		flags.enableGateSensor = true;
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
	unsigned int packetCount;
	byte crc;
	
	char name[12]; 
	
	NameCommand()
	{
		id = NAME_CMD_ID;
		packetCount = 0;
		crc = 0x00;
	}

};
typedef struct NameCommand NameCommand;

//With this msgpart this command request/contains the SHA signature id
#define IDENTITY_GET_SIGNATURE_MSG_PART 10

struct __attribute__ ((packed)) IdentityCommand
{

	char id;
	unsigned int packetCount;
	byte crc;
	
	byte msgPart;
	
	char payload[12]; 
	
	IdentityCommand()
	{
		id = IDENTITY_CMD_ID;
		packetCount = 0;
		crc = 0x00;
		msgPart = 0;
	}

};
typedef struct IdentityCommand IdentityCommand;