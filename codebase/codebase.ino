#include <Servo.h>

#define MOTOR_PIN 2
#define STEERING_PIN 3

Servo motor_servo;
Servo steering_servo;

void set_speed(float mps) {
  motor_servo.writeMicroseconds(mps);
}

void set_steering(float angle) {
  steering_servo.write(angle);
}


void setup() {

  delay(5 * 1000);
  
  motor_servo.attach(MOTOR_PIN);
  steering_servo.attach(STEERING_PIN);
}

void loop() {}
