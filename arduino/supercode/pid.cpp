#include <PID_v1.h>
#include <math.h>

#include "pid.h"
#include "car.h"
#include "log.h"

// velocity to velocity
pid_tuning pid_velocity_tuning = { 0.03984, 79.88, 0 };
pid_io velocity_io = { 0, 0, 0.5 };

PID velocity_pid DEFAULT_PID(velocity_io, pid_velocity_tuning);

void pid_init() {
    velocity_pid.SetMode(AUTOMATIC);
    velocity_pid.SetOutputLimits(0, 500);
    velocity_pid.SetSampleTime(100);
    
}

void pid_set_tuning(const pid_tuning& tuning, PID& controller) {
   controller.SetTunings(tuning.Kp, tuning.Ki, tuning.Kd); 
}

void pid_update() {
    car_measurements car = *(car_get_measurements());

    velocity_io.in = car_get_velocity(); 
    if (car_get_current_servo_value() > 105 || car_get_current_servo_value() < 85) {
        velocity_io.set = 0.2;  
    }
    else {
        velocity_io.set = 0.3;
    }

    // compute new velocity
    velocity_pid.Compute();

    // set velocity from controller
    //car_set_velocity(velocity_io.out - angle_vel_io.out + 1500);
    car_set_velocity(velocity_io.out + 1500);
}
