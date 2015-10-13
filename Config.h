// Configure the 3DRacers Library

//Enable the console shell: connect with a Serial Console (ie: from the Arduino IDE) to isse commands: (default: true)
#define SHELL_ENABLED true

//Enable startup message when console enabled: (default: true)
#define STARTUP_LOG true

//Detailed debug trough the serial console: (default: false)
#define SERIAL_DEBUG false

//Detailed network debug. For a standard network debug use the /netdebug command in the Console Shell. (default: false)
#define SERIAL_DEBUG_NET false

//Startup motors debug. To check for the motors wiring use the /motorscheck command in the Console Shell. (default: false)
#define MOTORS_DEBUG false















//MACRO DEFINITION - DO NOT EDIT!

#if SERIAL_DEBUG
	#define DEBUG if(Serial) Serial->print
	#define DEBUGln if(Serial) Serial->println
#else
	#define DEBUG 
	#define DEBUGln 
#endif

#if SERIAL_DEBUG_NET
	#define DEBUGNET Serial->print
	#define DEBUGNETln Serial->println
#else
	#define DEBUGNET 
	#define DEBUGNETln 
#endif

#if STARTUP_LOG && (SERIAL_DEBUG || SERIAL_DEBUG_NET || SHELL_ENABLED)
	#define STARTLOG Serial->print
	#define STARTLOGln Serial->println
	#define STARTLOGF(S) Serial->print(F(S))
	#define STARTLOGFln(S) Serial->println(F(S))
#else
	#define STARTLOG 
	#define STARTLOGln 
	#define STARTLOGF 
	#define STARTLOGFln 
#endif

#if SHELL_ENABLED
	#define SHELLOUT Serial->print
	#define SHELLOUTln Serial->println
	#define SHELLOUTF(S) Serial->print(F(S))
	#define SHELLOUTFln(S) Serial->println(F(S))
#else
	#define SHELLOUT 
	#define SHELLOUTln 
	#define SHELLOUTF
	#define SHELLOUTFln 
#endif