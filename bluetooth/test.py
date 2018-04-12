from sys import exit
import struct
from bluetooth import discover_devices, BluetoothSocket, RFCOMM

def find_device_mac(name):
    devices = discover_devices(lookup_names=True)
    print(devices)
    for dev in devices:
        if dev[1] == name:
            return dev

if __name__ == '__main__':
    devices = discover_devices(lookup_names=True)

    default_addr = "20:13:09:13:36:96"

    addr, name = find_device_mac('HC-06')

    addr = default_addr

    print(addr)

    socket = BluetoothSocket(RFCOMM)
    socket.connect((addr, 1))
    print("Connected to {} on channel {}".format(addr, 1))

    while True:
        inp = input()
        if inp == "quit":
            break
        print("Sending {}".format(inp))
        print(socket.send(struct.Struct('B').pack(0b11110000)))
        print("Sent data")
