#ifndef CAR_H
#define CAR_H

#include <Arduino.h>
#include <Servo.h>
#include <math.h>

void car_init(int speed_pins[], Servo *motor_servo, Servo *steering_servo);
void car_update_velocity();
void car_set_velocity(double mps);
void car_set_steering(double mps);
double car_get_velocity();
double car_get_steering();

#endif
