#include <Servo.h>
#include <PID_v1.h>
#include <Thread.h>
#include <ThreadController.h>

#include "config.h"
#include "log.h"
#include "bluetooth.h"
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

// bluetooth ctrl struct
car_ctrl_packet_result ccpr;

// should car be running
bool motor_running = false;


Thread bluetooth_send_thread;
Thread car_update_thread;
Thread pid_update_thread;
Thread printing_thread;
ThreadController tctrl;

void printing() {
    //DEBUG_LOGLN(car_get_steering()); 
    DEBUG_LOGLNS(car_get_sensor_state(), BIN);
    //DEBUG_LOGLN(car_get_velocity());
    //DEBUG_LOGLN(car_get_sensor_angle() * (180 / M_PI));
    DEBUG_LOGLN(car_get_sensor_distance());
}

void setup() {
    // setup serial port for usb
    SERIAL_PORT.begin(9600);
    if (USE_BLUETOOTH) {
        BLUETOOTH_PORT.begin(9600);
    }

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

    if (false) {
        if (USE_BLUETOOTH){ 
            DEBUG_LOGLN("waiting for bluetooth...");
            bluetooth_init(&BLUETOOTH_PORT);
            while (ccpr.type != car_ctrl_packet_type::PWR_ON && 
                   ccpr.complete != true) {
                bluetooth_serial_read(ccpr);
            }
            DEBUG_LOGLN("bluetoooth remote start, recieved.");
        }
    }

    DEBUG_LOGLN("initializing car...");
    car_init(speed_pins, sensor_array, &motor_servo, &steering_servo);
    
    DEBUG_LOGLN("initializing pids...");
    pid_init();

    // setup all the threads
    DEBUG_LOGLN("initializing threads...");

    printing_thread.enabled = true;
    printing_thread.setInterval(1000);
    printing_thread.onRun(printing);

    if (USE_BLUETOOTH) {
        bluetooth_send_thread.enabled = false;
        bluetooth_send_thread.setInterval(50);
        bluetooth_send_thread.onRun(bluetooth_thread_send);
    }

    car_update_thread.enabled = true;
    car_update_thread.setInterval(50);
    car_update_thread.onRun(car_update_velocity);

    pid_update_thread.enabled = true;
    pid_update_thread.setInterval(50);
    pid_update_thread.onRun(pid_update);

    tctrl = ThreadController();
    if (USE_BLUETOOTH) {
        tctrl.add(&bluetooth_send_thread);
    }

    tctrl.add(&car_update_thread);
    tctrl.add(&printing_thread);
    tctrl.add(&pid_update_thread);

    DEBUG_LOGLN("finish...");
    digitalWrite(LED_BUILTIN, HIGH);

    car_set_steering(0);
    
    DEBUG_FLUSH();
}

void loop() {
    if (false) {
        if (USE_BLUETOOTH) {
            bluetooth_serial_read(ccpr);
            if (ccpr.complete)
                ccpr_parse_packet(ccpr);
        }
    }

    tctrl.run();
}
