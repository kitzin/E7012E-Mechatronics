#include "pid.h"

void set_pid_tuning(const pid_tuning& tuning, PID& controller) {
   controller.SetTunings(tuning.Kp, tuning.Ki, tuning.Kd); 
}
