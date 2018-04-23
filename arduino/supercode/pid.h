#ifndef PID_H
#define PID_H

#include <PID_v1.h>

#define DEFAULT_PID(in, out, tuning) 

typedef struct {
    double Kp;
    double Ki;
    double Kd;
} pid_tuning;


void pid_init();
void pid_set_tuning(const pid_tuning& tuning, PID& controller);
void pid_update();
#endif
