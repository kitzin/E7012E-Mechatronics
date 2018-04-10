#include <Servo.h>

constexpr int servo_pin = 3;
constexpr int blink_pin = 2;
constexpr int blink_hz = 10;

int angle = 60;

Servo turn_servo;

void setup() {
  // put your setup code here, to run once:
  pinMode(blink_pin, OUTPUT);
  Serial.begin(9600);
  turn_servo.attach(servo_pin);
  turn_servo.write(angle);
}

void loop() {
  // put your main code here, to run repeatedly:
  /*digitalWrite(blink_pin, HIGH);
  delay(1000 / (blink_hz * 2));
  digitalWrite(blink_pin, LOW);
  delay(1000 / (blink_hz * 2));

  for(int i = 0; i<50; ++i) {
    turn_servo.write(i);
    delay(25);
  }
  for(int i = 50; i>=0; --i) {
    turn_servo.write(i);
    delay(25);
  }*/
  turn_servo.write(angle);
  delay(100);
}
