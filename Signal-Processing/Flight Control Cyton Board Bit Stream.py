import random
import struct
import time
import sys
import numpy as np
import serial
from serial.tools import list_ports

for port in list_ports.comports():
    print(port.device)
    print(port.description)
    print()
ser = serial.Serial("/dev/cu.usbserial-DP05IFC2", 115200, timeout=0.1)
time.sleep(0.2)
START_BYTE = 0xA0
STOP_BYTES = [0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF] 
ser.write(b"b")
ser.reset_input_buffer()
ser.reset_output_buffer()
ser.write(b"v")
time.sleep(0.5)
ser.write(b"b")
def int24_to_bytes(value):
    
    #Convert signed 24-bit integer to 3 bytes (big-endian)

    if value < 0:
        value += 1 << 24
    return value.to_bytes(3, byteorder='big')
"""
def generate_cyton_packet(sample_num):
    packet = bytearray()
    # Start byte
    packet.append(START_BYTE)
    
    # Sample number
    packet.append(sample_num & 0xFF)
    
    # 8 EEG channels (24-bit signed)
    for _ in range(8):
        eeg_value = random.randint(-2**23, 2**23 - 1)
        packet.extend(int24_to_bytes(eeg_value))
    
    # 3 AUX channels (16-bit signed)
    for _ in range(3):
        aux_value = random.randint(-32768, 32767)
        packet.extend(struct.pack('>h', aux_value))  # big-endian int16
    
    # Stop byte
    packet.append(random.choice(STOP_BYTES))

    return bytes(packet)

def cyton_byte_stream(packet_rate_hz=250, max_packets=None):
    sample_num = 0
    interval = 1 / packet_rate_hz
    packets_sent = 0

    while True:
        if max_packets is not None and packets_sent >= max_packets:
            return  # stops generator

        packet = generate_cyton_packet(sample_num)
        sample_num = (sample_num + 1) % 256
        packets_sent += 1
        byte_interval = interval / 33

        for byte in packet:
            yield byte
            time.sleep(byte_interval)
"""



buffer = bytearray()

validPackets = []
EEG1 = []
EEG2 = []
EEG3 = []
EEG4 = []
EEG5 = []
EEG6 = []
EEG7 = []
EEG8 = []
packets_received = 0
while True:
    if ser.is_open == False:
        break
    n = ser.in_waiting; chunk = ser.read(n or 1)
    print(f"Read {len(chunk)} bytes: {chunk.hex()}")
    if not chunk:
        continue
        print ("No data received, waiting...")
    buffer.extend(chunk)
    while True:
        if len(buffer) >= 33:
            i = buffer.find(START_BYTE)
            if i > 0:
                del buffer[:i]
            elif i == -1:
                buffer.clear()
                break
            if buffer[0] == 0xA0 and buffer[32] in STOP_BYTES:
                packet = buffer[:33]
                del buffer[:33]
                validPackets.append(packet)
                print("Valid packet:", packet.hex())
                packets_received += 1
                if packets_received >= 100:
                    ser.write(b"s")
                    ser.close()
                    break
            else:
                buffer.pop(0)
        else:
            break
       
print("Num of Valid Packets: ",  len(validPackets))

for i in validPackets:
    EEG1.append(int.from_bytes(i[2:5], byteorder='big', signed=True))
    EEG2.append(int.from_bytes(i[5:8], byteorder='big', signed=True))
    EEG3.append(int.from_bytes(i[8:11], byteorder='big', signed=True))
    EEG4.append(int.from_bytes(i[11:14], byteorder='big', signed=True))
    EEG5.append(int.from_bytes(i[14:17], byteorder='big', signed=True))
    EEG6.append(int.from_bytes(i[17:20], byteorder='big', signed=True))
    EEG7.append(int.from_bytes(i[20:23], byteorder='big', signed=True))
    EEG8.append(int.from_bytes(i[23:26], byteorder='big', signed=True))

print("EEG1: ", EEG1)
print("EEG2: ", EEG2)
print("EEG3: ", EEG3)
print("EEG4: ", EEG4)
print("EEG5: ", EEG5)
print("EEG6: ", EEG6)
print("EEG7: ", EEG7)
print("EEG8: ", EEG8)