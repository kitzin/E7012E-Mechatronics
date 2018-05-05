#ifndef CAR_H
#define CAR_H

#include <Arduino.h>
#include <Servo.h>
#include <math.h>

typedef struct {
    float length;
    float distance_to_sensor;
    float sensor_distances[7];
} car_measurements;

void car_init(int speed_pins[], int sensor_pins[], Servo *motor_servo, Servo *steering_servo);
void car_update_velocity();
void car_set_velocity(float mps);
void car_set_steering(float mps);
float car_get_velocity();
float car_get_steering();
float car_get_sensor_angle();
float car_get_sensor_distance();
car_measurements* car_get_measurements();

#endif
