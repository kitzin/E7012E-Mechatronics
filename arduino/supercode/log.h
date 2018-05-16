#include "config.h"

#define DEBUG_LOG(s) if (DEBUG) { SERIAL_PORT.print(s); if (USE_BLUETOOTH) BLUETOOTH_PORT.print(s); }
#define DEBUG_LOGS(s, b) if (DEBUG) { SERIAL_PORT.print(s, b); if (USE_BLUETOOTH) BLUETOOTH_PORT.print(s, b); }
#define DEBUG_LOGLN(s) if (DEBUG) { DEBUG_LOG(s); SERIAL_PORT.println(""); if (USE_BLUETOOTH) BLUETOOTH_PORT.println(""); }
#define DEBUG_LOGLNS(s, b) if (DEBUG) { DEBUG_LOGS(s, b); SERIAL_PORT.println(""); if (USE_BLUETOOTH) BLUETOOTH_PORT.println(""); }
#define DEBUG_FLUSH() { if (USE_BLUETOOTH) BLUETOOTH_PORT.flush(); SERIAL_PORT.flush(); }
