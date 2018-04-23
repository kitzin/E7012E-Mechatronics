#include "bluetooth.h"
#include "car.h"
HardwareSerial *bluetooth_serial;

char bluetooth_buffer[MAX_BUFFER_SIZE];
char* buffer_start;
char* buffer_end;
char* buffer_current_read_pos;
char* buffer_current_max_data_pos;
int bytes_in_buffer = 0;

void bluetooth_init(HardwareSerial *s) {
    bluetooth_serial = s;
    bluetooth_serial->begin(9600);
    memset(bluetooth_buffer,0,MAX_BUFFER_SIZE);

    //Init buffer counters
    buffer_start = (char *)&bluetooth_buffer;
    buffer_end = (char *)(&bluetooth_buffer + MAX_BUFFER_SIZE - 1);

    buffer_current_read_pos = buffer_start;
    buffer_current_max_data_pos = buffer_start;
}

void bluetooth_thread_send(){
    if(bytes_in_buffer > 0){
        bluetooth_serial->write(*buffer_current_read_pos);
        bytes_in_buffer--;
        if(buffer_current_read_pos == buffer_end){
            buffer_current_read_pos = buffer_start;
        }
        else{
            buffer_current_read_pos++;
        }
    }
    else{
        return;
    }
    //for (int i=0; i<64;i++) {
    //}
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

void bluetooth_send(const char *const data, int len) {
    for (int i = 0; i<len; ++i) {
       if(bytes_in_buffer >= MAX_BUFFER_SIZE){
            Serial.println("BUFFER_OVERFLOW");
            break;
       }
       else{
           if(buffer_current_max_data_pos == buffer_end){
               buffer_current_max_data_pos = buffer_start;
           }
           else if(buffer_current_max_data_pos == buffer_start && bytes_in_buffer == 0){
                //This is the starting point
           }
           else{
               buffer_current_max_data_pos++;
           }
           *buffer_current_max_data_pos = data[i];
           bytes_in_buffer++;
       }
    }

    /*
    for (int i = 0; i<len; ++i) {
        bluetooth_serial->write(data[i]);
    }
    */
}

void bluetooth_send_string(char str[]){
    int size = 0;
    while(true){
        if (str[size] == 0)
            break;
        else
            size += 1;
    }
    bluetooth_serial->write("<");
    bluetooth_serial->write(size);
    for (int i = 0; i < size; i++){
        bluetooth_serial->write(str[i]);
    }
}

void ccpr_reset(car_ctrl_packet_result& ccpr) {
    ccpr.complete = false;
}

void ccpr_parse_packet(car_ctrl_packet_result &ccpr) {
    static bool motor_running = true;
    uint32_t speed = 0;
    uint32_t steering = 0;
    char speed_buf[19];
    switch (ccpr.type) {
        case SET_SPEED:
            if(!motor_running)
                return;
            speed = bluetooth_serial->read();
            speed = speed<<8 | bluetooth_serial->read();
            speed = speed<<8 | bluetooth_serial->read();
            speed = speed<<8 | bluetooth_serial->read();
            
            Serial.println("Speed set to: ");
            Serial.println(String(speed));
            car_set_velocity(speed);
            break;
        case GET_SPEED:
            bluetooth_serial->write(car_get_velocity());
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
            car_set_velocity(1500);
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
