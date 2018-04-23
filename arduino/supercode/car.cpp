#include "car.h"

#define CAR_WHEEL_RADIUS (64.06/2)
#define CAR_MAX_STEERING_ANGLE (25*(M_PI/180))
#define CAR_STEERING_SERVO_MAX 80
#define CAR_STEERING_SERVO_MIN 40

#define ANGLE_VELOCITY_PREVIOUS_MAX 5

Servo *car_motor_servo;
Servo *car_steering_servo;

double angle_velocity;
double current_steering;

int pulse_right_count = 0;
int pulse_left_count = 0;

int angle_velocity_count = 0;
double previous_angle_velocities[ANGLE_VELOCITY_PREVIOUS_MAX];

void calculate_angle_velocity(double *angle_velocity, double previous_velocities[]) {
    double current = 0;
    for (int i = 0; i<ANGLE_VELOCITY_PREVIOUS_MAX; ++i) {
        current += previous_velocities[i];
    }
    *angle_velocity = current / ANGLE_VELOCITY_PREVIOUS_MAX;
}

void car_speedsensor_left_pulse() {
    ++pulse_left_count;
}

void car_speedsensor_right_pulse() {
    ++pulse_right_count;
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
    if (angle > CAR_MAX_STEERING_ANGLE)
        angle = CAR_MAX_STEERING_ANGLE;
    if (angle < -CAR_MAX_STEERING_ANGLE)
        angle = -CAR_MAX_STEERING_ANGLE;

    angle += CAR_MAX_STEERING_ANGLE;
    int servo_diff = CAR_STEERING_SERVO_MAX - CAR_STEERING_SERVO_MIN;
    int servo_value = servo_diff + (angle / (CAR_MAX_STEERING_ANGLE * 2)) * servo_diff;
    
    car_steering_servo->write(servo_value);
    current_steering = angle;
}

void car_update_velocity(){
    static unsigned long previous_time = 0;
    unsigned long current_time = micros(); 
    if (previous_time == 0) {
        previous_time = current_time;
        return;
    }

    double current_angle_velocity =  ((pulse_left_count + pulse_right_count) / 2 * (M_PI / 2)) / (current_time - previous_time);

    previous_angle_velocities[angle_velocity_count++] = current_angle_velocity;
    calculate_angle_velocity(&angle_velocity, previous_angle_velocities);

    previous_time = current_time;
    pulse_left_count = pulse_right_count = 0;

    if (angle_velocity_count > ANGLE_VELOCITY_PREVIOUS_MAX-1)
        angle_velocity_count = 0;
}

double car_get_velocity() {
    return angle_velocity * CAR_WHEEL_RADIUS * 1000;
}

double car_get_steering() {
    return current_steering;
}
