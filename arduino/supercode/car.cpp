#include "car.h"

#define CAR_WHEEL_RADIUS (64.06/2)
#define CAR_MAX_STEERING_ANGLE (25*(M_PI/180))
#define CAR_STEERING_SERVO_MAX 80
#define CAR_STEERING_SERVO_MIN 40

#define ANGLE_VELOCITY_PREVIOUS_MAX 3

Servo *car_motor_servo;
Servo *car_steering_servo;

double left_angle_velocity;
double right_angle_velocity;

double current_velocity;
double current_steering;

double previous_left_angle_velocities[ANGLE_VELOCITY_PREVIOUS_MAX];
double previous_right_angle_velocities[ANGLE_VELOCITY_PREVIOUS_MAX];

int angle_velocity_right_count = 0;
int angle_velocity_left_count = 0;

void calculate_angle_velocity(double *angle_velocity, double previous_velocities[]) {
    double current = 0;
    for (int i = 0; i<ANGLE_VELOCITY_PREVIOUS_MAX; ++i) {
        current += previous_velocities[i];
    }
    *angle_velocity = current / ANGLE_VELOCITY_PREVIOUS_MAX;
}

void car_speedsensor_left_pulse() {
    static unsigned long previous_time = 0; 
    unsigned long current_time = micros(); 
    if (previous_time == 0) {
        previous_time = current_time;
        return;
    }


    double current_angle_velocity = M_PI / (current_time - previous_time);
    previous_left_angle_velocities[angle_velocity_left_count++] = current_angle_velocity;
    calculate_angle_velocity(&left_angle_velocity, previous_left_angle_velocities);

    previous_time = current_time;

    if (angle_velocity_left_count > 2)
        angle_velocity_left_count = 0;
}

void car_speedsensor_right_pulse() {
    static unsigned long previous_time = 0; 
    unsigned long current_time = micros(); 
    if (previous_time == 0) {
        previous_time = current_time;
        return;
    }

    double current_angle_velocity = M_PI / (current_time - previous_time);
    previous_right_angle_velocities[angle_velocity_right_count++] = current_angle_velocity;
    calculate_angle_velocity(&right_angle_velocity, previous_right_angle_velocities);

    previous_time = current_time;

    if (angle_velocity_right_count > 2)
        angle_velocity_right_count = 0;
}

void car_init(int speed_pins[], Servo *m_servo, Servo *s_servo) {
    attachInterrupt(digitalPinToInterrupt(speed_pins[0]), car_speedsensor_left_pulse, FALLING);
    attachInterrupt(digitalPinToInterrupt(speed_pins[1]), car_speedsensor_right_pulse, FALLING);

    car_motor_servo = m_servo;
    car_steering_servo = s_servo;
}


void car_set_velocity(double mps) {
    int speed = (int)mps;
    if (speed > 2000)
        speed = 2000;
    if (speed < 1000)
        speed = 1000;
    car_motor_servo->writeMicroseconds(speed);
}

void car_set_steering(double angle) {
    Serial.println(angle);
    if (angle > CAR_MAX_STEERING_ANGLE)
        angle = CAR_MAX_STEERING_ANGLE;
    if (angle < -CAR_MAX_STEERING_ANGLE)
        angle = -CAR_MAX_STEERING_ANGLE;

    angle += CAR_MAX_STEERING_ANGLE;
    int servo_diff = CAR_STEERING_SERVO_MAX - CAR_STEERING_SERVO_MIN;
    int servo_value = servo_diff + (angle / (CAR_MAX_STEERING_ANGLE * 2)) * servo_diff;
    
    Serial.println(servo_value);
    car_steering_servo->write(servo_value);
    current_steering = angle;
}

double car_get_velocity() {
    Serial.print(left_angle_velocity * CAR_WHEEL_RADIUS * 1000);
    Serial.print(" : ");
    Serial.println(right_angle_velocity *  CAR_WHEEL_RADIUS * 1000);
    return ((left_angle_velocity + right_angle_velocity) / 2) * CAR_WHEEL_RADIUS * 1000;
}

double car_get_steering() {
    return current_steering;
}
