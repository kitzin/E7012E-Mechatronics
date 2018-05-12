#ifndef CAR_H
#define CAR_H

#include <Arduino.h>
#include <Servo.h>
#include <math.h>

#define CAR_WHEEL_RADIUS (64.06/2)
#define CAR_MAX_STEERING_ANGLE (25*(M_PI/180))


#define CAR_MAX_STEERING_ANGLE_RIGHT (35*(M_PI/180))
#define CAR_MAX_STEERING_ANGLE_LEFT (22*(M_PI/180))
#define CAR_STEERING_SERVO_MAX 90 
#define CAR_STEERING_SERVO_MIDDLE 65 
#define CAR_STEERING_SERVO_MIN 40

#define ANGLE_VELOCITY_PREVIOUS_MAX 5
#define SENSOR_ARRAY_PREVIOUS_MAX 5

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
uint8_t car_get_sensor_state();

#endif
