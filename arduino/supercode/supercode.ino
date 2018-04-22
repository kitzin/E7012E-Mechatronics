#include <Servo.h>
#include <PID_v1.h>
#include <Thread.h>
#include <ThreadController.h>

#include "config.h"
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

pid_tuning pid_velocity_tuning = { 1, 2, 3 };
pid_tuning pid_angle_tuning = { 1, 2, 3 };

double velocity_in, velocity_out, velocity_set, angle_in, angle_out, angle_set;

PID velocity_pid(&velocity_in,
                 &velocity_out,
                 &velocity_set,
                 pid_velocity_tuning.Kp,
                 pid_velocity_tuning.Ki,
                 pid_velocity_tuning.Kd,
                 DIRECT);
PID angle_pid(&angle_in,
              &angle_out,
              &angle_set,
              pid_angle_tuning.Kp,
              pid_angle_tuning.Ki,
              pid_angle_tuning.Kd,
              DIRECT);

Thread test_thread;
Thread bluetooth_send_thread;
ThreadController tctrl;

void testf() {
    static float last_velocities[2] = {3.f, 1.f};
    static int count_velocities = 0;
    float velocity = car_get_velocity();
    float angle = car_get_steering();
    long time = millis();
    
    if ( last_velocities[0] == last_velocities[1] && last_velocities[0] == velocity){
         
    }
    else {
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

void setup() {

    // setup serial port for usb
    Serial.begin(9600);

    // setup pins for sensor array
    Serial.println("setting up pins for sensors...");
    for (const auto& pin : sensor_array) {
        pinMode(pin, INPUT);
    }
    // setup pins for speed sensors
    for (const auto& pin : speed_pins) {
        pinMode(pin, INPUT);
    }

    // wait for motor to start (Controller sends out 1V during startup)
    pinMode(MOTOR_PWR_PIN, INPUT);
    Serial.print("waiting for motor to start...");
    while(analogRead(MOTOR_PWR_PIN) < MOTOR_PWR_THRESHOLD);
    Serial.println("ok."); 
    
    Serial.println("set motor output to low...");
    pinMode(MOTOR_PIN, OUTPUT);
    digitalWrite(MOTOR_PIN, LOW);


    // attach steering and motor to servo
    Serial.println("attaching servos...");
    steering_servo.attach(STEERING_PIN);
    motor_servo.attach(MOTOR_PIN);

    if (USE_BLUETOOTH){ 
        Serial.println("waiting for bluetooth...");
        bluetooth_init(&BLUETOOTH_PORT);
        while (ccpr.type != car_ctrl_packet_type::PWR_ON && 
               ccpr.complete != true) {
            bluetooth_serial_read(ccpr);
        }
        Serial.println("bluetoooth remote start, recieved.");
    }

    Serial.println("initializing car...");
    car_init(speed_pins, &motor_servo, &steering_servo);
    
    Serial.println("starting pids...");
    velocity_pid.SetMode(AUTOMATIC);
    angle_pid.SetMode(AUTOMATIC);

    Serial.println("finish...");


    test_thread.enabled = true;
    test_thread.setInterval(10);
    test_thread.onRun(testf);

    bluetooth_send_thread.enabled = true;
    bluetooth_send_thread.setInterval(50);
    bluetooth_send_thread.onRun(bluetooth_thread_send);

    tctrl = ThreadController();
    tctrl.add(&test_thread);
    tctrl.add(&bluetooth_send_thread);
}

void loop() {
    if (USE_BLUETOOTH) {
        bluetooth_serial_read(ccpr);
        if (ccpr.complete)
            ccpr_parse_packet(ccpr);
    }

    velocity_pid.Compute();
    angle_pid.Compute(); 
    
    tctrl.run();
}

