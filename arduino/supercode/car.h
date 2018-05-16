#ifndef CAR_H
#define CAR_H

#include <Arduino.h>
#include <Servo.h>
#include <math.h>

#define CAR_WHEEL_RADIUS (64.06/2)

#define CAR_MAX_STEERING_ANGLE_RIGHT (30 * (M_PI/180))
#define CAR_MAX_STEERING_ANGLE_LEFT (25 * (M_PI/180))
#define CAR_STEERING_SERVO_RIGHT 70
#define CAR_STEERING_SERVO_MIDDLE 95 
#define CAR_STEERING_SERVO_LEFT 125

#define ANGLE_VELOCITY_PREVIOUS_MAX 5
//#define SENSOR_ARRAY_PREVIOUS_MAX 3
#define SENSOR_ARRAY_PREVIOUS_MAX 1 

typedef struct {
    float length;
    float distance_to_sensor;
    float sensor_distances[6];
} car_measurements;

void car_init(int speed_pins[], int sensor_pins[], Servo *motor_servo, Servo *steering_servo);
void car_update_velocity();
void car_set_velocity(float mps);
void car_set_steering(float mps);

//simple steering
void car_set_simple_steering(float angle);
float car_get_velocity();
float car_get_steering();
float car_get_sensor_angle();
float car_get_sensor_distance();
car_measurements* car_get_measurements();
uint8_t car_get_sensor_state();

#endif

