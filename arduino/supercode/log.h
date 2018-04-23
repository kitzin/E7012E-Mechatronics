#include "config.h"

#define DEBUG_LOG(s) if (DEBUG) { SERIAL_PORT.print(s); }
#define DEBUG_LOGLN(s) if (DEBUG) { DEBUG_LOG(s); SERIAL_PORT.println(""); }
