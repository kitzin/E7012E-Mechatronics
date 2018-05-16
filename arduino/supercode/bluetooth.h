#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdint.h>
#include <Arduino.h>

#define MAX_BYTE_RATE 100
#define MAX_BUFFER_SIZE 10000

enum car_ctrl_packet_type : uint8_t {
    INFO            = 0x0,
    PWR_ON          = 0x1,
    PWR_OFF         = 0x2,
    SET_SPEED       = 0x3,
    GET_SPEED       = 0x4, 
    GET_STEERING    = 0x5,
    SET_STEERING    = 0x6,
    GET_ARRAY       = 0x7,
};

const int expected_packet_size[] = {0, 0, 0, 4, 0, 0, 4, 0};

typedef struct {
    bool complete = false;
    car_ctrl_packet_type type;
    uint8_t size;
} car_ctrl_packet_result;

void bluetooth_thread_send();
void bluetooth_init(HardwareSerial *serial);
void bluetooth_serial_read(car_ctrl_packet_result& ccpr);
void bluetooth_send_string(char str[]);
void bluetooth_send(const char *const data, int len);

void ccpr_parse_packet(car_ctrl_packet_result& ccpr);
void ccpr_reset(car_ctrl_packet_result& ccpr);
#endif

