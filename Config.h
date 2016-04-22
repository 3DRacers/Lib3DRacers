// Configure the 3DRacers Library

//Reduced falsh memory footprint of the library: (default: false)
#define REDUCED_FEATURES false

//Enable the console shell: connect with a Serial Console (ie: from the Arduino IDE) to isse commands: (default: true)
#define ENABLE_SHELL true

//Enable startup message when console enabled: (default: false)
#define ENABLE_STARTUP_LOG false

//Detailed debug trough the serial console: (default: false)
#define ENABLE_SERIAL_DEBUG false					&& REDUCED_FEATURES

//Detailed network debug. Disable STARTUP_LOG if using this. For a standard network debug use the /netdebug command in the Console Shell. (default: false)
#define ENABLE_SERIAL_DEBUG_NET false				&& REDUCED_FEATURES

//Startup motors debug. To check for the motors wiring use the /motorscheck command in the Console Shell. (default: false)
#define MOTORS_DEBUG false















//MACRO DEFINITION - DO NOT EDIT!

#if REDUCED_FEATURES
	#define SHELL_ENABLED false
	#define STARTUP_LOG false
	#define SERIAL_DEBUG false
	#define SERIAL_DEBUG_NET false
#else
	#define SHELL_ENABLED ENABLE_SHELL
	#define STARTUP_LOG ENABLE_STARTUP_LOG
	#define SERIAL_DEBUG ENABLE_SERIAL_DEBUG
	#define SERIAL_DEBUG_NET ENABLE_SERIAL_DEBUG_NET
#endif

#if !REDUCED_FEATURES && SERIAL_DEBUG
	#define DEBUG if(Serial) Serial->print
	#define DEBUGln if(Serial) Serial->println
#else
	#define DEBUG 
	#define DEBUGln 
#endif

#if !REDUCED_FEATURES && SERIAL_DEBUG_NET
	#define DEBUGNET Serial->print
	#define DEBUGNETln Serial->println
#else
	#define DEBUGNET 
	#define DEBUGNETln 
#endif

#if !REDUCED_FEATURES && STARTUP_LOG && (SERIAL_DEBUG || SERIAL_DEBUG_NET || SHELL_ENABLED)
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

#if !REDUCED_FEATURES && SHELL_ENABLED
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