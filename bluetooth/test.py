from sys import exit
from time import sleep
import struct
from bluetooth import discover_devices, BluetoothSocket, RFCOMM

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

    socket = BluetoothSocket(RFCOMM)
    socket.connect((addr, 1))
    print("Connected to {} on channel {}".format(addr, 1))

    
    while True:
        inp = input(">")
        if inp == "quit":
            break
        
        converted = 0;
        try:
            convert = int(inp)
        except ValueError as e:
            continue 
        sendByte(socket,convert)
