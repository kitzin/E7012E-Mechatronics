#ifndef PID_H
#define PID_H

#include <PID_v1.h>

#define DEFAULT_PID(in, out, tuning) 

typedef struct {
    double Kp;
    double Ki;
    double Kd;
} pid_tuning;


void set_pid_tuning(const pid_tuning& tuning, PID& controller);

#endif
