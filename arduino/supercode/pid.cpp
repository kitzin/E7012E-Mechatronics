#include <PID_v1.h>
#include <math.h>

#include "pid.h"
#include "car.h"
#include "log.h"

// velocity to velocity
pid_tuning pid_velocity_tuning = { 0.03984, 79.88, 0 };
pid_io velocity_io = { 0, 0, 0.5 };
// angle to angle
pid_tuning pid_angle_tuning = { 0.0, 0.09, 0.03 };
pid_io angle_io = { 0, 0, 0 };
// angle to velocitiy
pid_tuning pid_anglevelocity_tuning = { 0.03984, 79.88, 0 };
pid_io angle_vel_io = { 0, 0, 0 };

PID velocity_pid DEFAULT_PID(velocity_io, pid_velocity_tuning);
PID angle_pid DEFAULT_PID(angle_io, pid_angle_tuning);
PID angle_velocity_pid DEFAULT_PID(angle_vel_io, pid_anglevelocity_tuning);

void pid_init() {
    velocity_pid.SetMode(AUTOMATIC);
    velocity_pid.SetOutputLimits(0, 500);
    velocity_pid.SetSampleTime(100);
    
    angle_pid.SetMode(AUTOMATIC);
    angle_pid.SetOutputLimits(-CAR_MAX_STEERING_ANGLE_RIGHT, CAR_MAX_STEERING_ANGLE_LEFT);
    velocity_pid.SetSampleTime(100);
    
    angle_velocity_pid.SetMode(AUTOMATIC);
    angle_velocity_pid.SetOutputLimits(0, 500);
    velocity_pid.SetSampleTime(100);
}

void pid_set_tuning(const pid_tuning& tuning, PID& controller) {
   controller.SetTunings(tuning.Kp, tuning.Ki, tuning.Kd); 
}

void pid_update() {
    car_measurements car = *(car_get_measurements());

    velocity_io.in = car_get_velocity(); 
    angle_io.in = -car_get_sensor_angle();
    angle_vel_io.in = -abs(car_get_sensor_angle());

    if (abs(car_get_sensor_distance()) > abs(car.sensor_distances[2])) {
        velocity_io.set = 0.2;  
    }
    else {
        velocity_io.set = 0.2;
    }

    // compute all controller things
    velocity_pid.Compute();
    angle_pid.Compute(); 
    //angle_velocity_pid.Compute();

    // set velocity from controller
    //car_set_velocity(velocity_io.out - angle_vel_io.out + 1500);
    car_set_velocity(velocity_io.out + 1500);
    
    float steering_angle = 1.3 * atan2(
            2 * car.length * car_get_sensor_distance(),
            pow(car.length + car.distance_to_sensor, 2) + pow(car_get_sensor_distance(), 2));
    
    // set steering angle from controller
    car_set_steering(angle_io.out + steering_angle);

    //uint8_t state = car_get_sensor_state();
    /*
    float steering_angle;
    float sensor_distance = car_get_sensor_distance();
    if (sensor_distance <= car.sensor_distances[1]){
        steering_angle = -30 * (M_PI/180);
    }
    else if (sensor_distance <= car.sensor_distances[2]) {
        steering_angle = -15 * (M_PI/180);
    }
    else if (sensor_distance >= car.sensor_distances[4]) {
        steering_angle = 15 * (M_PI/180);
    }
    else if (sensor_distance >= car.sensor_distances[5]) {
        steering_angle = 25 * (M_PI/180);
    }
    else {
        steering_angle = 0;
    }

    car_set_steering(steering_angle);
    */
}
