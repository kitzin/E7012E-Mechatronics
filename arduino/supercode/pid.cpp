#include <PID_v1.h>

#include "pid.h"

pid_tuning pid_velocity_tuning = { 1, 2, 3 };
pid_tuning pid_angle_tuning = { 1, 2, 3 };

double velocity_in, velocity_out, velocity_set, angle_in, angle_out, angle_set;

PID velocity_pid(&velocity_in,
                 &velocity_out,
                 &velocity_set,
                 pid_velocity_tuning.Kp,
                 pid_velocity_tuning.Ki,
                 pid_velocity_tuning.Kd,
                 DIRECT);
PID angle_pid(&angle_in,
              &angle_out,
              &angle_set,
              pid_angle_tuning.Kp,
              pid_angle_tuning.Ki,
              pid_angle_tuning.Kd,
              DIRECT);

void pid_init() {
    velocity_pid.SetMode(AUTOMATIC);
    angle_pid.SetMode(AUTOMATIC);
}

void pid_set_tuning(const pid_tuning& tuning, PID& controller) {
   controller.SetTunings(tuning.Kp, tuning.Ki, tuning.Kd); 
}

void pid_update() {
    velocity_pid.Compute();
    angle_pid.Compute(); 
}
