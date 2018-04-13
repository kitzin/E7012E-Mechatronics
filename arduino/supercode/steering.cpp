#include "steering.h"

void set_steering(Servo &servo, int angle) {
    servo.write(angle);
}
