#include <Servo.h>

#define MOTOR_PIN 7

Servo motor_servo;

void setup() {
  // put your setup code here, to run once:
  motor_servo.attach(MOTOR_PIN);
}

void loop() {
  motor_servo.writeMicroseconds(1300);
  delay(100);
}
