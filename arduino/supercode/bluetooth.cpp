#include "bluetooth.h"


void bluetooth_init(HardwareSerial& serial) {
    serial.begin(9600);
}

void bluetooth_wait_for_cmd(HardwareSerial& serial, car_ctrl_packet_result r) {

}

void bluetooth_serial_read(car_ctrl_packet_result& ccpr, HardwareSerial& serial) {
    static uint8_t packet_size = 0;
    static uint8_t packet_type = 0;
    static bool waiting_for_packet = false;

    if (waiting_for_packet) {
        if (serial.available() >= packet_size) {
            ccpr.type = convert_binary_type(packet_type);
            ccpr.size = packet_size;
            ccpr.complete = true;
            waiting_for_packet = false;
        }
    } else {
        if (serial.available() >= 2) {
            packet_type = serial.read();
            if (packet_type > (sizeof(expected_packet_size)/sizeof(*expected_packet_size))-1) {
               return;
            }
            packet_size = expected_packet_size[packet_type];

            ccpr.complete = false;
            waiting_for_packet = true;
        }
    }
}

car_ctrl_packet_type convert_binary_type(uint8_t &type) {
    switch (type) {
        case 0x1:
            return car_ctrl_packet_type::PWR_ON;
        case 0x2:
            return car_ctrl_packet_type::PWR_OFF;        
        case 0x3:
            return car_ctrl_packet_type::SET_SPEED;
        case 0x4:
            return car_ctrl_packet_type::GET_SPEED;        
        case 0x5:
            return car_ctrl_packet_type::GET_STEERING;
        case 0x6:
            return car_ctrl_packet_type::SET_STEERING;        
        case 0x7:
            return car_ctrl_packet_type::GET_ARRAY;        
        case 0x8:
            return car_ctrl_packet_type::INFO;
    }
} 
