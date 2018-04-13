#include <Servo.h>

#include "motor.h"
#include "steering.h"

#define STEERING_PIN 3
#define MOTOR_PIN 4
#define START_SEQUENCE 0b11110000

// Servos for motor and steering
Servo motor_servo;
Servo steering_servo;



void setup() {
    // setup serial port for usb and bluetooth
    Serial.begin(9600);
    Serial1.begin(9600);

    // wait for motor to start
    Serial.println("waiting five seconds for motor to start...");
    delay(5000);

    // attach steering and motor to servo
    steering_servo.attach(STEERING_PIN);
    //motor_servo.attach(MOTOR_PIN, 1000, 2000);

    Serial.println("setting default servo values");
    // do stuff

    Serial.print("waiting for start sequence");
    while (false) {
        if (Serial1.available() > 0) {
            uint8_t seq =  Serial1.read();
            if (seq == START_SEQUENCE) {
                break;
            }
        }
        delay(500);
        Serial.print('.');
        Serial.flush();
    }
    
    Serial.println();
    Serial.println("got start sequence...");
    delay(1000);
}

void loop() {
    //set_steering(steering_servo, 0); // 0 full right, 50, full left
    //delay(1000);
    //set_steering(steering_servo,50);
    //delay(1000);
    //set_steering(steering_servo,0);
    //delay(1000);
    //set_steering(steering_servo,50);
    //delay(1000);
    //set_steering(steering_servo,0);
    
    
    //set_speed(motor_servo,1500);
    //Serial.println("1500");
    //delay(100);
      
    motor_servo.attach(MOTOR_PIN);
    for (int i=1600; i<1900; i = i+50){
        Serial.println(i);
        set_speed(motor_servo,i);
        delay(500);
    }
    
    for (int i=1900; i>1600; i = i-50){
        Serial.println(i);
        set_speed(motor_servo,i);
        delay(500);
    }

    for (int i=1400; i>1100; i = i-50){
        Serial.println(i);
        set_speed(motor_servo,i);
        delay(500);
    }
    for (int i=1100; i<1400; i = i+50){
        Serial.println(i);
        set_speed(motor_servo,i);
        delay(500);
    }
    
    motor_servo.detach();
    //set_speed(motor_servo,1550);
    
    Serial.print(".");
    delay(1000);
    Serial.print(".");
    delay(1000);
    Serial.print(".");
    delay(1000);
    Serial.print(".");
    delay(1000);
    Serial.print(".\n");
    delay(1000);
    /*
    Serial.println("Reseting Motor");
    set_speed(motor_servo, 1500); // 1000 - 1500 forward, 1500-2000 backwards
    delay(1000);
    Serial.println("1600");
    set_speed(motor_servo, 1600); // 1000 - 1500 forward, 1500-2000 backwards
    delay(1000);
    Serial.println("1700");
    set_speed(motor_servo, 1700); // 1000 - 1500 forward, 1500-2000 backwards
    delay(1000);
    Serial.println("1800");
    set_speed(motor_servo, 1800); // 1000 - 1500 forward, 1500-2000 backwards
    delay(1000);
    Serial.println("1700");
    set_speed(motor_servo, 1700); // 1000 - 1500 forward, 1500-2000 backwards
    delay(1000);
    Serial.println("1600");
    set_speed(motor_servo, 1600); // 1000 - 1500 forward, 1500-2000 backwards
    delay(1000);
    */
}

