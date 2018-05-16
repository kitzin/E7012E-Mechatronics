#include <Servo.h>

#define MOTOR_PIN 3

#define MOTOR_PWR_PIN A3
#define MOTOR_PWR_THRESHOLD 150

Servo motor_servo;

void setup() {
    Serial.begin(9600);
    //steering_servo.attach(STEERING_PIN);
    //steering_servo.write(43);
    pinMode(5,OUTPUT);

    pinMode(MOTOR_PWR_PIN, INPUT);
    Serial.println("waiting for motor to start...");
    while(analogRead(MOTOR_PWR_PIN) < MOTOR_PWR_THRESHOLD) delay(100);
    Serial.println("ok.");  
    
    Serial.println("set motor output to low...");
    pinMode(MOTOR_PIN, OUTPUT);
    digitalWrite(MOTOR_PIN, LOW);

    // attach steering and motor to servo
    Serial.println("attaching motor servo...");
    motor_servo.attach(MOTOR_PIN);
}

int value = 0;
void loop() {
    if (Serial.available()){
        value = Serial.parseInt();
        if (value > 2000  || value < 1000 ){
            Serial.println("Invalid speed");
            return;
        }
        Serial.println(value);
        motor_servo.writeMicroseconds(value);
        //digitalWrite(5,HIGH);
    }
}
