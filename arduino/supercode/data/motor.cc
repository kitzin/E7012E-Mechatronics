#include "motor.h"

void set_speed(Servo &servo, int mps) {
    servo.writeMicroseconds(mps);
}
