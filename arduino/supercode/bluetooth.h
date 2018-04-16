#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdint.h>
#include <Arduino.h>
//#include <array>


enum car_ctrl_packet_type : uint8_t {
    PWR_ON          = 0x1,
    PWR_OFF         = 0x2,
    SET_SPEED       = 0x3,
    GET_SPEED       = 0x4, 
    GET_STEERING    = 0x5,
    SET_STEERING    = 0x6,
    GET_ARRAY       = 0x7,
    INFO            = 0x8,
};

int expected_packet_size[] = {0, 0, 4, 0, 4, 0, 0, 0};

typedef struct {
    bool complete = true;
    car_ctrl_packet_type type;
    uint8_t size;
} car_ctrl_packet_result;

void bluetooth_init(HardwareSerial &serial);

void bluetooth_wait_for_cmd(HardwareSerial& serial, car_ctrl_packet_result r);


void bluetooth_serial_read(car_ctrl_packet_result& ccpr, HardwareSerial& serial);

car_ctrl_packet_type convert_binary_type(uint8_t &type);
#endif
