#!/usr/bin/env python3
import time
import serial

serial_port = 0;

class Controller:
    def __init__(self,serial_interface):
        if not serial_interface.is_open:
            ser.open()
        self.serial_interface = serial_interface

    def run(self):
        print("Welcome, plz control your car: (h for help)\n")
        text = ""
        while(True):
            text = input(">")
            converted = 0;
            try:
                convert = int(text)
            except ValueError as e:
                continue 
            self.sendByte(hex(convert))

    def sendByte(self,byte):
        print(byte)
        self.serial_interface.write(byte);
if __name__ == "__main__":
    ser = serial.Serial()
    ser.baudrate = 9600
    ser.port = '/dev/ttyACM1'
    ser.open()
    controller = Controller(ser);
