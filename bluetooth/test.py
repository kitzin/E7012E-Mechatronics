from time import sleep
import struct
from bluetooth import discover_devices, BluetoothSocket, RFCOMM
import _thread



def find_device_mac(name):
    devices = discover_devices(lookup_names=True)
    print(devices)
    for dev in devices:
        if dev[1] == name:
            return dev

def askForInput(prompt,minimum,maximum):
    while True:
        inp = input(prompt)
        if inp == "quit":
            break
        converted = 0;
        try:
            convert = int(inp)
        except ValueError as e:
            continue
        if minimum <= convert and  convert <= maximum:
            convertAndSendDecimal(socket,convert)
            return
        else:
            print("To big")

def convertAndSendDecimal(socket,number):
    comparitor = 0b11111111 << 8*3;
    shifter = 8
    count = 3
    while(comparitor > 0 ):
        sendByte(socket,((number & comparitor) >> 8*count))
        comparitor = comparitor >> shifter
        count -= 1

def sendByte(socket,byte):
    print("Sending {}".format(byte))
    print(socket.send(struct.Struct('B').pack(byte)))
    print("Sent data")

def checkForData(threadName,socket):
    print("%s %s" % (threadName, "reporting."))
    data_chunk = b''
    waiting_for_data = False
    next_chunk_size = 0
    while True:
        data = socket.recv(1024)
        #data = socket.recv(1)
        if data and not waiting_for_data:
            data_chunk = b''
            waiting_for_data = True
            print("%s: %s" % (threadName, data))
            next_chunk_size = int.from_bytes(data, byteorder='little')
            print(next_chunk_size)
        elif data:
            data_chunk += data
            print(data_chunk)
            if len(data_chunk) >= next_chunk_size:
                print("%s: %s" % (threadName, data_chunk.decode("utf-8", "ignore")))
                waiting_for_data = False

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

    print("Starting listening thread.")
    _thread.start_new_thread( checkForData, ("Listing_thread", socket,))

    while True:
        inp = input(">")
        if inp == "quit":
            break
        converted = 0;
        try:
            convert = int(inp)
        except ValueError as e:
            continue
        if convert == 3:
            sendByte(socket,convert)
            askForInput("Speed: ",1500,1700)
        else:
            sendByte(socket,convert)


