from time import sleep
import time
import struct
from bluetooth import discover_devices, BluetoothSocket, RFCOMM
import _thread
import pathlib

import os
from os import path
from datetime import datetime


def find_device_mac(name):
    devices = discover_devices(lookup_names=True)
    print(devices)
    for dev in devices:
        if dev[1] == name:
            return dev


def sendByte(socket,byte):
    print("Sending {}".format(byte))
    print(socket.send(struct.Struct('B').pack(byte)))
    print("Sent data")

if __name__ == '__main__':
    print("Starting...")
    #devices = discover_devices(lookup_names=True)
    #print(devices)

    default_addr = "20:13:09:13:36:96"
    #print(default_addr);
    #sleep(10);
    
    #addr, name = find_device_mac('HC-06')

    addr = default_addr

    print(addr)
    def connect(addr):
        socket = BluetoothSocket(RFCOMM)
        while True:
            try:
                socket.connect((addr, 1))
                break
            except Exception as e:
                print("Failed to connect trying, again in 1 sec.")
                time.sleep(1)
                
        print("Connected to {} on channel {}".format(addr, 1))
        return socket

    socket = connect(addr)
    buf = ""
    count_print = 0
    while True:
        data = None
        while True:
            try:
                data = socket.recv(1024)
                break
            except:
                socket.close()
                socket = connect(addr)

        if data:
            for b in data:
                b = chr(b)
                if b == "\n":
                    print(buf)
                    buf = ""
                else:
                    buf += b 
