#import random
#import struct
#import sys
import time
import serial
from serial.tools import list_ports
from EEGRawFilter import filter_sample_live
WARMUP_PACKETS = 100#250 * 16  # 16 seconds at 250 Hz
START_BYTE = 0xA0
STOP_BYTES = [0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF]
## CHECKING FOR AVAILABLE SERIAL PORTS - DEBUGGING PURPOSES 
for port in list_ports.comports():
    print(port.device)
    print(port.description)
    print()
## CYTON BOARD SERIAL CONNECTION
ser = serial.Serial("/dev/cu.usbserial-DP05IFC2", 115200, timeout=0.1)
time.sleep(0.2)
ser.reset_input_buffer()
ser.reset_output_buffer()
ser.write(b"v")
time.sleep(0.5)
ser.write(b"b")
## END OF CYTON BOARD SERIAL CONNECTION

## HELPER FUNCTION TO CONVERT 24-BIT SIGNED INTEGER TO 3 BYTES
def int24_to_bytes(value):
    #Convert signed 24-bit integer to 3 bytes (big-endian)
    if value < 0:
        value += 1 << 24
    return value.to_bytes(3, byteorder='big')

## MAIN LOOP TO READ FROM SERIAL PORT AND PROCESS PACKETS
buffer = bytearray()
SCALE_UV = (4.5 / (24 * (2**23 - 1))) * 1e6
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
    #print(f"Read {len(chunk)} bytes: {chunk.hex()}")
    if not chunk:
        #print ("No data received, waiting...")
        continue
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
                channels_raw = [
                    int.from_bytes(packet[2:5],   'big', signed=True),
                    int.from_bytes(packet[5:8],   'big', signed=True),
                    int.from_bytes(packet[8:11],  'big', signed=True),
                    int.from_bytes(packet[11:14], 'big', signed=True),
                    int.from_bytes(packet[14:17], 'big', signed=True),
                    int.from_bytes(packet[17:20], 'big', signed=True),
                    int.from_bytes(packet[20:23], 'big', signed=True),
                    int.from_bytes(packet[23:26], 'big', signed=True),
                ]
                filtered = [
                    filter_sample_live(raw * SCALE_UV, ch)
                    for ch, raw in enumerate(channels_raw)
                ]
            
                if packets_received > WARMUP_PACKETS:
                    EEG1.append(filtered[0])
                    EEG2.append(filtered[1])
                    EEG3.append(filtered[2])
                    EEG4.append(filtered[3])
                    EEG5.append(filtered[4])
                    EEG6.append(filtered[5])
                    EEG7.append(filtered[6])
                    EEG8.append(filtered[7])
                    validPackets.append(filtered)
                    n = len(validPackets)
                    print(f"\n{'─'*44}")
                    print(f"  PKT #{n:>4}   (total received: {packets_received})")
                    print(f"{'─'*44}")
                    for ch, val in enumerate(filtered, 1):
                        bar_len = int(min(abs(val), 100) / 100 * 20)
                        bar = ('█' * bar_len).ljust(20)
                        sign = '+' if val >= 0 else '-'
                        print(f"  CH{ch}  {sign}{abs(val):>8.2f} µV  |{bar}|")
                    print(f"{'─'*44}")
                packets_received += 1
                #if packets_received >= 1000:
                    #ser.write(b"s")
                   # ser.close()
                   # break
            else:
                buffer.pop(0)
                time.sleep(0.01)
        else:
          break

print("Num of Valid Packets: ",  len(validPackets))

print("EEG1: ", EEG1)
print("EEG2: ", EEG2)
print("EEG3: ", EEG3)
print("EEG4: ", EEG4)
print("EEG5: ", EEG5)
print("EEG6: ", EEG6)
print("EEG7: ", EEG7)
print("EEG8: ", EEG8)