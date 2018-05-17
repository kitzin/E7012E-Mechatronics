#include <Servo.h>
#include <PID_v1.h>

#include "config.h"
#include "log.h"
#include "car.h"
#include "pid.h"

// Servos for motor and steering
Servo motor_servo;
Servo steering_servo;

// Array sensor pins
int sensor_array[] = {
    ARRAY_PIN_1, ARRAY_PIN_2, ARRAY_PIN_3, ARRAY_PIN_4,
    ARRAY_PIN_5, ARRAY_PIN_6 };

// Speed sensor pins
int speed_pins[] = {
    SPEED_PIN_LEFT, SPEED_PIN_RIGHT };

void printing() {
    DEBUG_LOGLN(car_get_current_servo_value()); 
    //DEBUG_LOGLNS(128 + car_get_sensor_state(), BIN);
    //DEBUG_LOGLN(car_get_velocity());
    //DEBUG_LOGLN(car_get_sensor_angle() * (180 / M_PI));
    //DEBUG_LOGLN(car_get_sensor_distance());
}

unsigned long time_last_pid = 0;
unsigned long time_last_car_update_vel = 0;
unsigned long time_last_car_update_angle = 0;
unsigned long time_since_print = 0;

unsigned long timeout_pid = 50;
unsigned long timeout_car_update_vel = 50;
unsigned long timeout_car_update_angle = 10;
unsigned long timeout_print = 1000;

void setup() {
    // setup serial port for usb
    SERIAL_PORT.begin(9600);

    // setup pins for sensor array
    DEBUG_LOGLN("setting up pins for sensors...");
    for (const auto& pin : sensor_array) {
        pinMode(pin, INPUT);
    }
    // setup pins for speed sensors
    for (const auto& pin : speed_pins) {
        pinMode(pin, INPUT);
    }
    
    //Enabling the internal led
    pinMode(LED_BUILTIN, OUTPUT);

    // attach steering servo
    DEBUG_LOGLN("attaching steering servo...");
    steering_servo.attach(STEERING_PIN);

    // wait for motor to start (Controller sends out 1V during startup)
    pinMode(MOTOR_PWR_PIN, INPUT);
    DEBUG_LOG("waiting for motor to start...");
    while(analogRead(MOTOR_PWR_PIN) < MOTOR_PWR_THRESHOLD) delay(100);
    DEBUG_LOGLN("ok.");  
    
    DEBUG_LOGLN("set motor output to low...");
    pinMode(MOTOR_PIN, OUTPUT);
    digitalWrite(MOTOR_PIN, LOW);

    // attach steering and motor to servo
    DEBUG_LOGLN("attaching motor servo...");
    motor_servo.attach(MOTOR_PIN);

    DEBUG_LOGLN("initializing car...");
    car_init(speed_pins, sensor_array, &motor_servo, &steering_servo);
    
    DEBUG_LOGLN("initializing pids...");
    pid_init();

    DEBUG_LOGLN("finish...");
    digitalWrite(LED_BUILTIN, HIGH);

	car_set_simple_steering(95);
	
    DEBUG_FLUSH();
}

void loop() {
	if (millis() - time_last_pid >= timeout_pid){
		time_last_pid = millis();
		pid_update();
	}

	if (millis() - time_last_car_update_angle >= timeout_car_update_angle){
		time_last_car_update_angle = millis();
		car_update_servo_angle();
	}

	if (millis() - time_last_car_update_vel >= timeout_car_update_vel){
		time_last_car_update_vel = millis();
		car_update_velocity();
	}

	if (millis() - time_since_print >= timeout_print){
		time_since_print = millis();
		printing();
	}
}
