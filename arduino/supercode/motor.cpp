#include "motor.h"

void set_speed(Servo &servo, int mps) {
    // max speed = 6.64V
    servo.writeMicroseconds(mps);
}

