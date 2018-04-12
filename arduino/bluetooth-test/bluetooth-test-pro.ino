#include <Servo.h>

#define DEBUG_LOG(X) Serial.println(String("[DEBUG]").concat(X))

#define STEERING_PIN 2
#define MOTOR_PIN 3

// Servos for motor and steering
Servo motor_servo;
Servo steering_servo;

void setup() {
    // setup serial port for usb and bluetooth
    Serial.begin(9600);
    Serial1.begin(9600);

    // wait for motor to start
    DEBUG_LOG("waiting five seconds for motor to start...");
    delay(5000);

    // attach steering and motor to servo
    steering_pin.attach(STEERING_PIN);
    motor_servo.attach(MOTOR_PIN);
}

void loop() {
    Serial.println("----");
    Serial.println((int)Serial1.available());
    Serial.println((int)Serial5.available());
    delay(1000);
    // Read user input if available.
    if (Serial.available()){
        delay(10); // The delay is necessary to get this working!
        Serial.write(Serial.read());
    }
}
