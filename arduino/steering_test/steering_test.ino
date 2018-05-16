#include <Servo.h>
#define STEERING_PIN 5 
#define IS_LOOPING false 
Servo steering_servo;

void setup() {
    Serial.begin(9600);
    steering_servo.attach(STEERING_PIN);
    pinMode(5,OUTPUT);
    
}

int value = 0;
void loop() {
    if (IS_LOOPING){
        for (int i = 43; i<=75; i++) {
            Serial.print("Value: ");
            Serial.print(i);
            Serial.println("");
            steering_servo.write(i);
            delay(100);
        }
        for (int i = 75; i >= 43; i--) {
            Serial.print("Value: ");
            Serial.print(i);
            Serial.println("");
            steering_servo.write(i);
            delay(100);
        }
    }
    else{
       if (Serial.available()){
            value = Serial.parseInt();
            if (value > 180 || value < 0){
                Serial.println("Invalid angle");
                return;
            }
            Serial.println(value);
            steering_servo.write(value);
            //digitalWrite(5,HIGH);
        }
    }
}
