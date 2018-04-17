#include "bluetooth.h"
#include "car.h"

HardwareSerial *bluetooth_serial;

void bluetooth_init(HardwareSerial *s) {
    bluetooth_serial = s;
    bluetooth_serial->begin(9600);
}

void bluetooth_serial_read(car_ctrl_packet_result& ccpr) {
    static uint8_t packet_size = 0;
    static uint8_t packet_type = 0;
    static bool waiting_for_packet = false;

    ccpr_reset(ccpr);

    if (waiting_for_packet) {
        if (bluetooth_serial->available() >= packet_size) {
            ccpr.type = static_cast<car_ctrl_packet_type>(packet_type);
            ccpr.size = packet_size;
            ccpr.complete = true;
            waiting_for_packet = false;
        }
    } else {
        if (bluetooth_serial->available() >= 1) {
            packet_type = bluetooth_serial->read();
            packet_size = expected_packet_size[packet_type];
            ccpr.complete = false;
            waiting_for_packet = true;
        }
    }
}

void ccpr_reset(car_ctrl_packet_result& ccpr) {
    ccpr.complete = false;
}

void ccpr_parse_packet(car_ctrl_packet_result &ccpr) {
    static bool motor_running = true;
    uint32_t speed = 0;
    uint32_t steering = 0;
    switch (ccpr.type) {
        case SET_SPEED:
            if(!motor_running)
                return;
            speed = bluetooth_serial->read();
            speed = speed<<8 | bluetooth_serial->read();
            speed = speed<<8 | bluetooth_serial->read();
            speed = speed<<8 | bluetooth_serial->read();
            
            Serial.println(speed);
            break;
        case GET_SPEED:
            break;
        case SET_STEERING:
            if(!motor_running)
                return;
            steering = bluetooth_serial->read();
            steering = steering<<8 | bluetooth_serial->read();
            steering = steering<<8 | bluetooth_serial->read();
            steering = steering<<8 | bluetooth_serial->read();

            Serial.println(steering);
            break;
        case GET_STEERING:
            break;
        case PWR_OFF:
            motor_running = false;
            Serial.println("motor off");
            break;
        case PWR_ON:
            motor_running = true;
            Serial.println("motor on");
            break;
        case GET_ARRAY:
            break;
        default:
            break;
    }
}
