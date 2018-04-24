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
    ARRAY_PIN_5, ARRAY_PIN_6, ARRAY_PIN_7 };

// Speed sensor pins
int speed_pins[] = {
    SPEED_PIN_LEFT, SPEED_PIN_RIGHT };

// bluetooth ctrl struct
car_ctrl_packet_result ccpr;

// should car be running
bool motor_running = false;


Thread test_thread;
Thread bluetooth_send_thread;
Thread car_update_thread;
Thread pid_update_thread;
Thread printing_thread;
ThreadController tctrl;

void testf() {
    static float last_velocities[2] = {3.f, 1.f};
    static int count_velocities = 0;
    float velocity = car_get_velocity();
    float angle = car_get_steering();
    long time = millis();
    
    if ( velocity != 0.f){
        bluetooth_send("|", 1);
        bluetooth_send((char*)&time, sizeof(time));
        bluetooth_send((char*)&velocity, sizeof(velocity));
        bluetooth_send((char*)&angle, sizeof(angle));
    }

    last_velocities[count_velocities++] = velocity;
    if(count_velocities > 1){
        count_velocities = 0;
    }
}

void printing() {
    DEBUG_LOGLN(car_get_velocity());    
}

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

    // attach steering servo
    DEBUG_LOGLN("attaching steering servo...");
    steering_servo.attach(STEERING_PIN);
    steering_servo.write(60);

    // wait for motor to start (Controller sends out 1V during startup)
    pinMode(MOTOR_PWR_PIN, INPUT);
    DEBUG_LOG("waiting for motor to start...");
    while(analogRead(MOTOR_PWR_PIN) < MOTOR_PWR_THRESHOLD);
    DEBUG_LOGLN("ok."); 
    
    DEBUG_LOGLN("set motor output to low...");
    pinMode(MOTOR_PIN, OUTPUT);
    digitalWrite(MOTOR_PIN, LOW);


    // attach steering and motor to servo
    DEBUG_LOGLN("attaching motor servo...");
    motor_servo.attach(MOTOR_PIN);

    if (USE_BLUETOOTH){ 
        DEBUG_LOGLN("waiting for bluetooth...");
        bluetooth_init(&BLUETOOTH_PORT);
        while (ccpr.type != car_ctrl_packet_type::PWR_ON && 
               ccpr.complete != true) {
            bluetooth_serial_read(ccpr);
        }
        DEBUG_LOGLN("bluetoooth remote start, recieved.");
    }

    DEBUG_LOGLN("initializing car...");
    car_init(speed_pins, &motor_servo, &steering_servo);
    
    DEBUG_LOGLN("initializing pids...");
    pid_init();

    // setup all the threads
    DEBUG_LOGLN("initializing threads...");
    test_thread.enabled = true;
    test_thread.setInterval(10);
    test_thread.onRun(testf);

    printing_thread.enabled = true;
    printing_thread.setInterval(1000);
    printing_thread.onRun(printing);

    bluetooth_send_thread.enabled = true;
    bluetooth_send_thread.setInterval(50);
    bluetooth_send_thread.onRun(bluetooth_thread_send);

    car_update_thread.enabled = true;
    car_update_thread.setInterval(50);
    car_update_thread.onRun(car_update_velocity);

    pid_update_thread.enabled = true;
    pid_update_thread.setInterval(10);
    pid_update_thread.onRun(pid_update);

    tctrl = ThreadController();
    tctrl.add(&test_thread);
    tctrl.add(&bluetooth_send_thread);
    tctrl.add(&car_update_thread);
    tctrl.add(&printing_thread);
    tctrl.add(&pid_update_thread);

    DEBUG_LOGLN("finish...");
}

void loop() {
    if (USE_BLUETOOTH) {
        bluetooth_serial_read(ccpr);
        if (ccpr.complete)
            ccpr_parse_packet(ccpr);
    }

    
    tctrl.run();
}

