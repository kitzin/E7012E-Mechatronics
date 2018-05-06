#include <PID_v1.h>
#include <math.h>

#include "pid.h"
#include "car.h"
#include "log.h"

// velocity to velocity
pid_tuning pid_velocity_tuning = { 0.03984, 79.88, 0 };
pid_io velocity_io = { 0, 0, 1 };
// angle to angle
pid_tuning pid_angle_tuning = { 0, 0.3, 0.003 };
pid_io angle_io = { 0, 0, 0 };
// angle to velocitiy
pid_tuning pid_anglevelocity_tuning = { 0.03984, 79.88, 0 };
pid_io angle_vel_io = { 0, 0, 0 };

PID velocity_pid DEFAULT_PID(velocity_io, pid_velocity_tuning);
PID angle_pid DEFAULT_PID(angle_io, pid_angle_tuning);
PID angle_velocity_pid DEFAULT_PID(angle_vel_io, pid_anglevelocity_tuning);

void pid_init() {
    velocity_pid.SetMode(AUTOMATIC);
    angle_pid.SetMode(AUTOMATIC);
    angle_velocity_pid.SetMode(AUTOMATIC);
}

void pid_set_tuning(const pid_tuning& tuning, PID& controller) {
   controller.SetTunings(tuning.Kp, tuning.Ki, tuning.Kd); 
}

void pid_update() {
    return;
    velocity_io.in = car_get_velocity(); 
    angle_io.in = car_get_sensor_angle();
    angle_vel_io.in = abs(car_get_sensor_angle());

    // compute all controller things
    velocity_pid.Compute();
    angle_pid.Compute(); 
    angle_velocity_pid.Compute();

    // set velocity from controller
    car_set_velocity(velocity_io.out - angle_vel_io.out + 1500);

    car_measurements car = *(car_get_measurements());

    float steering_angle = atan2(
            2 * car.length * car.distance_to_sensor,
            pow(car.length + car.distance_to_sensor, 2) + pow(car_get_sensor_distance(), 2));

    // set steering angle from controller
    car_set_steering(angle_io.out + steering_angle);
}
