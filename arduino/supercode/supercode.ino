#include <Servo.h>

#define STEERING_PIN 2
#define MOTOR_PIN 3
#define START_SEQUENCE 0b11110000

// Servos for motor and steering
Servo motor_servo;
Servo steering_servo;

void set_steering(int angle) {
    steering_servo.write(angle);
}

void set_speed(int mps) {
    motor_servo.writeMicroseconds(mps);
}

void setup() {
    // setup serial port for usb and bluetooth
    Serial.begin(9600);
    Serial1.begin(9600);

    // wait for motor to start
    Serial.println("waiting five seconds for motor to start...");
    delay(5000);

    // attach steering and motor to servo
    steering_servo.attach(STEERING_PIN);
    motor_servo.attach(MOTOR_PIN);

    Serial.println("setting default servo values");
    // do stuff

    Serial.print("waiting for start sequence");
    while (true) {
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
    delay(1000000);
}

void loop() {
    Serial.println("----");
    Serial.println((int)Serial1.available());
    delay(1000);
    // Read user input if available.
    if (Serial.available()){
        delay(10); // The delay is necessary to get this working!
        Serial.write(Serial.read());
    }
}

