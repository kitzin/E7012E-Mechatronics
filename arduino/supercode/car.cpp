#include "car.h"
#include "log.h"

#include <math.h>


Servo *car_motor_servo;
Servo *car_steering_servo;

float angle_velocity;

int pulse_right_count = 0;
int pulse_left_count = 0;

int angle_velocity_count = 0;
float previous_angle_velocities[ANGLE_VELOCITY_PREVIOUS_MAX] = { 0 };

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
        speed = 1900;
    if (speed < 1500)
        speed = 1500;
    car_motor_servo->writeMicroseconds(speed);
} 

void car_set_simple_steering(float angle) {
	car_steering_servo->write(angle);
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

int get_servo_value_from_state(uint8_t state) {

    int servo_value = -1;
   switch(state) {
       case 0b100000:
           servo_value = 70;
           break;
       case 0b110000:
           servo_value = 70;
           break;
       case 0b111000:
           servo_value = 80;
           break;
       case 0b010000:
           servo_value = 80;
           break;
       case 0b011000:
           servo_value = 85;
           break;
       case 0b011100:
           servo_value = 90;
           break;
       case 0b001000:
           servo_value = 90;
           break;
       case 0b001100:
           servo_value = 95;
           break;
       case 0b001110:
           servo_value = 100;
           break;
       case 0b000100:
           servo_value = 100;
           break;
       case 0b000110:
           servo_value = 105; 
           break;
       case 0b000111:
           servo_value = 110;
           break;
       case 0b000010:
            servo_value = 110;
           break;
       case 0b000011:
           servo_value = 120;
           break;
       case 0b000001:
           servo_value = 120;
           break;

        default:
           servo_value = -1;
           break;
   }

   return servo_value;
}

int previous_servo_value = 0;
int car_get_current_servo_value() {
    return previous_servo_value;
}

void car_update_servo_angle() {
    uint8_t state = car_get_sensor_state();

    if(get_servo_value_from_state(state | previous_servo_value) != -1) {
        previous_servo_value = get_servo_value_from_state(state);
    }

    car_set_simple_steering(previous_servo_value);
}

car_measurements* car_get_measurements() {
    static car_measurements car_mes = { 0.263, 0.185, { -0.075, -0.035, -0.015, 0.015, 0.035, 0.075 } };
    return &car_mes;
}

uint8_t car_get_sensor_state() {
    uint8_t sensor_state = 0;
    for (int i = 0; i<6; ++i) {
        if (digitalRead(sensor_pins[i]) == HIGH) {
            bitSet(sensor_state, i);
        } else {
            bitClear(sensor_state, i);
        }
    } 
    return sensor_state;
}
