#ifndef PID_H
#define PID_H

#include <PID_v1.h>

#define DEFAULT_PID(io, tuning) (&io.in, &io.out, &io.set, tuning.Kp, tuning.Ki, tuning.Kd, DIRECT)

typedef struct {
    double Kp;
    double Ki;
    double Kd;
} pid_tuning;

typedef struct {
   double in;
   double out;
   double set; 
} pid_io;


void pid_init();
void pid_set_tuning(const pid_tuning& tuning, PID& controller);
void pid_update();
#endif
