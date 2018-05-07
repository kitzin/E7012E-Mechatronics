#include "car.h"
#include "log.h"

#include <math.h>

#define CAR_WHEEL_RADIUS (64.06/2)
#define CAR_MAX_STEERING_ANGLE (25*(M_PI/180))
#define CAR_STEERING_SERVO_MAX 80
#define CAR_STEERING_SERVO_MIN 40

#define ANGLE_VELOCITY_PREVIOUS_MAX 5

Servo *car_motor_servo;
Servo *car_steering_servo;

float angle_velocity;
float current_steering;

int pulse_right_count = 0;
int pulse_left_count = 0;

int angle_velocity_count = 0;

float previous_angle_velocities[ANGLE_VELOCITY_PREVIOUS_MAX];

int *sensor_pins;

void calculate_angle_velocity(float *angle_velocity, float previous_velocities[]) {
    float current = 0;
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

void car_init(int speed_pins[], int sense_pins[], Servo *m_servo, Servo *s_servo) {
    attachInterrupt(digitalPinToInterrupt(speed_pins[0]), car_speedsensor_left_pulse, FALLING);
    attachInterrupt(digitalPinToInterrupt(speed_pins[1]), car_speedsensor_right_pulse, FALLING);

    sensor_pins = sense_pins;

    car_motor_servo = m_servo;
    car_steering_servo = s_servo;
}

void car_set_velocity(float mps) {
    int speed = (int)mps;
    if (speed > 2000)
        speed = 2000;
    if (speed < 1000)
        speed = 1000;
    car_motor_servo->writeMicroseconds(speed);
} 

void car_set_steering(float angle) {
    if (angle > CAR_MAX_STEERING_ANGLE)
        angle = CAR_MAX_STEERING_ANGLE;
    if (angle < -CAR_MAX_STEERING_ANGLE)
        angle = -CAR_MAX_STEERING_ANGLE;

    angle += CAR_MAX_STEERING_ANGLE;
    int servo_diff = CAR_STEERING_SERVO_MAX - CAR_STEERING_SERVO_MIN;
    int servo_value = servo_diff + (angle / (CAR_MAX_STEERING_ANGLE * 2)) * servo_diff;
    
    car_steering_servo->write(servo_value);
    current_steering = angle - CAR_MAX_STEERING_ANGLE;
}

void car_update_velocity(){
    static unsigned long previous_time = 0;
    unsigned long current_time = micros(); 
    if (previous_time == 0) {
        previous_time = current_time;
        return;
    }

    //Could crash, divid by 0
    float current_angle_velocity =  ((pulse_left_count + pulse_right_count) / 2 * (M_PI / 2)) / (current_time - previous_time);

    previous_angle_velocities[angle_velocity_count++] = current_angle_velocity;
    calculate_angle_velocity(&angle_velocity, previous_angle_velocities);


    previous_time = current_time;
    pulse_left_count = pulse_right_count = 0;

    if (angle_velocity_count > ANGLE_VELOCITY_PREVIOUS_MAX - 1)
        angle_velocity_count = 0;
}

float car_get_velocity() {
    return angle_velocity * CAR_WHEEL_RADIUS * 1000;
}

float car_get_steering() {
    return current_steering;
}

// angle from car to line
float car_get_sensor_angle() {
    car_measurements car_m = *(car_get_measurements());
    return atan2(car_get_sensor_distance(), car_m.length + car_m.distance_to_sensor);
}

// distance from middle to line
float car_get_sensor_distance() {
    static float previous_distance = 0;
    
    car_measurements car_vals = *(car_get_measurements());

    float found_sensors[7] = { 0 };
    int found_index = 0;
    int mid = 3;

    bool ignore_left = false;
    bool ignore_right = false;

    bool sensor_found = false;

    uint8_t sensor_state = car_get_sensor_state();

    if (bitRead(sensor_state, mid) == 1) {
        found_sensors[found_index++] = car_vals.sensor_distances[mid];
        sensor_found = true;
    }
    for (int i=1; i<=3; ++i) {

        if (bitRead(sensor_state, mid + i)) {
            if (!ignore_right) {
                found_sensors[found_index++] = car_vals.sensor_distances[mid + i];
                sensor_found = true;
            }
        } else {
            if (sensor_found)
                ignore_right = true;
        }
        
        if (!ignore_left && bitRead(sensor_state, mid - i)) {
            if (!ignore_left) {
                found_sensors[found_index++] = car_vals.sensor_distances[mid - i];
                sensor_found = true;
            }
        } else {
            if (sensor_found)
                ignore_left = true;
        }

        if (ignore_left && ignore_right) {
            break;
        }
    }

    if (found_index == 0)
        return previous_distance;

    float distance = 0;
    for (int i = 0; i<=found_index; ++i) {
        distance += found_sensors[i];
    }

    previous_distance = distance /= found_index;

    return distance;
}

car_measurements* car_get_measurements() {
    static car_measurements car_mes = { 0.263, 0.185, { -0.15, -0.09, -0.04, 0, 0.04, 0.09, 0.15 } };
    return &car_mes;
}

uint8_t car_get_sensor_state() {
    uint8_t sensor_state = 0;
    for (int i = 0; i<7; ++i) {
        if (digitalRead(sensor_pins[i]) == HIGH) {
            bitSet(sensor_state, i);
        } else {
            bitClear(sensor_state, i);
        }
    } 
    return 0b1000000;
    //return sensor_state;
}
