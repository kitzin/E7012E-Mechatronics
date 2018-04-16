#include <Servo.h>
//#include <array>

#include "config.h"
#include "bluetooth.h"
#include "motor.h"
#include "steering.h"

#define START_SEQUENCE 0b11110000

// Servos for motor and steering
Servo motor_servo;
Servo steering_servo;

/*
std::array<int, 7> sensor_array = {
    ARRAY_PIN_1, ARRAY_PIN_2, ARRAY_PIN_3, ARRAY_PIN_4,
    ARRAY_PIN_5, ARRAY_PIN_6, ARRAY_PIN_7 };
*/
int sensor_array[] = {
    ARRAY_PIN_1, ARRAY_PIN_2, ARRAY_PIN_3, ARRAY_PIN_4,
    ARRAY_PIN_5, ARRAY_PIN_6, ARRAY_PIN_7 };

/*
std::array<int, 3> speed_pins = {
    SPEED_PIN_1, SPEED_PIN_2, SPEED_PIN_3 };
*/

int speed_pins[] = {
    SPEED_PIN_1, SPEED_PIN_2, SPEED_PIN_3 };

car_ctrl_packet_result ccpr;

void setup() {
    // setup serial port for usb and bluetooth
    Serial.begin(9600);
    if (USE_BLUETOOTH) {
        bluetooth_init(BLUETOOTH_PORT);
    }

    // setup pins for sensor array
    for (const auto& pin : sensor_array) {
        pinMode(pin, INPUT);
    }
    // setup pins for speed sensors
    for (const auto& pin : speed_pins) {
        pinMode(pin, INPUT);
    }

    pinMode(MOTOR_PIN, INPUT);

    // start motor with power transistor
    pinMode(MOTOR_PWR_PIN, OUTPUT);
    analogWrite(MOTOR_PWR_PIN, 255);
    
    // wait for motor to start
    Serial.print("waiting for motor to start...");
    while (digitalRead(MOTOR_PIN) == LOW);
    Serial.println("motor has started...");

    Serial.println("set motor output to low...");
    pinMode(MOTOR_PIN, OUTPUT);
    digitalWrite(MOTOR_PIN, LOW);


    // attach steering and motor to servo
    Serial.println("attaching servos...");
    steering_servo.attach(STEERING_PIN);
    motor_servo.attach(MOTOR_PIN);

    if (USE_BLUETOOTH){ 
            while (ccpr.type != car_ctrl_packet_type::PWR_ON 
                && ccpr.complete != true){
            bluetooth_serial_read(ccpr, BLUETOOTH_PORT);
        }
        /*
        car_ctrl_packet_result r;
        r.type = car_ctrl_packet_type::PWR_ON;
        bluetooth_wait_for_cmd(BLUETOOTH_PORT, r);
        */
    }
}

int value = 0;
uint32_t command_data = 0b0;
bool motor_running = false;
void loop() {
    bluetooth_serial_read(ccpr, BLUETOOTH_PORT);
     
    if (ccpr.complete) {
        switch (ccpr.type) {
            case SET_SPEED:
                if(!motor_running)
                    return;
                break;
            case GET_SPEED:
                break;
            case SET_STEERING:
                if(!motor_running)
                    return;
                break;
            case GET_STEERING:
                break;
            case PWR_OFF:
                break;
            case PWR_ON:
                break;
            case GET_ARRAY:
                break;
            default:
                break;
        }
    }

    delay(100);
    set_speed(motor_servo, 1200);
    Serial.println("Set motor servo to 1k");
    return;
    //set_steering(steering_servo, 0); // 0 full right, 50, full left
    /*
    for (int i=0; i<33; i++){
        Serial.println(i);
        set_steering(steering_servo,i);
        delay(100);
    }
    for (int i=33; i>0; i--){
        Serial.println(i);
        set_steering(steering_servo,i);
        delay(100);
    }*/
   
    if (Serial.available()){
        delay(10);
        //value = Serial.read();
        value = Serial.parseInt();
        Serial.println(value);
    }
    Serial.println("");
    Serial.print("Current: ");
    Serial.print(value);
    set_steering(steering_servo,value);
    delay(100);
    //16 == straight
    //33 == full left
    
    //set_steering(steering_servo,0);
    //delay(1000);
    //set_steering(steering_servo,50);
    //delay(1000);
    //set_steering(steering_servo,0);
    
    
    //set_speed(motor_servo,1500);
    //Serial.println("1500");
    //delay(100);
    /*
    motor_servo.attach(MOTOR_PIN);
    for (int i=1600; i<1900; i = i+50){
        Serial.println(i);
        set_speed(motor_servo,i);
        delay(500);
    }
    
    for (int i=1900; i>1600; i = i-50){
        Serial.println(i);
        set_speed(motor_servo,i);
        delay(500);
    }

    for (int i=1400; i>1100; i = i-50){
        Serial.println(i);
        set_speed(motor_servo,i);
        delay(500);
    }
    for (int i=1100; i<1400; i = i+50){
        Serial.println(i);
        set_speed(motor_servo,i);
        delay(500);
    }
    
    motor_servo.detach();
    */
    //set_speed(motor_servo,1550);
    /*
    for (int i=0;i<3;i++){
        Serial.print(".");
        delay(1000);
    } 
    Serial.println("");
    */
    /*
    Serial.println("Reseting Motor");
    set_speed(motor_servo, 1500); // 1000 - 1500 forward, 1500-2000 backwards
    delay(1000);
    Serial.println("1600");
    set_speed(motor_servo, 1600); // 1000 - 1500 forward, 1500-2000 backwards
    delay(1000);
    Serial.println("1700");
    set_speed(motor_servo, 1700); // 1000 - 1500 forward, 1500-2000 backwards
    delay(1000);
    Serial.println("1800");
    set_speed(motor_servo, 1800); // 1000 - 1500 forward, 1500-2000 backwards
    delay(1000);
    Serial.println("1700");
    set_speed(motor_servo, 1700); // 1000 - 1500 forward, 1500-2000 backwards
    delay(1000);
    Serial.println("1600");
    set_speed(motor_servo, 1600); // 1000 - 1500 forward, 1500-2000 backwards
    delay(1000);
    */
}

