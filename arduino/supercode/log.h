#include "config.h"

#define DEBUG_LOG(s) if (DEBUG) { SERIAL_PORT.print(s); }
#define DEBUG_LOGS(s, b) if (DEBUG) { SERIAL_PORT.print(s, b); }
#define DEBUG_LOGLN(s) if (DEBUG) { DEBUG_LOG(s); SERIAL_PORT.println(""); }
#define DEBUG_LOGLNS(s, b) if (DEBUG) { DEBUG_LOGS(s, b); SERIAL_PORT.println(""); }
