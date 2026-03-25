import random
import struct
import time
import serial
from serial.tools import list_ports
from EEGRawFilter import filter_sample_live
WARMUP_PACKETS =  10  # 16 seconds at 250 Hz
START_BYTE = 0xA0
STOP_BYTES = [0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF]
# Replace serial port with simulator
def int24_to_bytes(value):
    
    #Convert signed 24-bit integer to 3 bytes (big-endian)

    if value < 0:
        value += 1 << 24
    return value.to_bytes(3, byteorder='big')
class SimulatedSerial:
    def __init__(self):
        self._buffer = bytearray()
        self._sample_num = 0
        self.is_open = True

    def _generate_packet(self):
        packet = bytearray()
        packet.append(START_BYTE)
        packet.append(self._sample_num & 0xFF)
        for _ in range(8):
            eeg_value = random.randint(-2**23, 2**23 - 1)
            packet.extend(int24_to_bytes(eeg_value))
        for _ in range(3):
            aux_value = random.randint(-32768, 32767)
            packet.extend(struct.pack('>h', aux_value))
        packet.append(random.choice(STOP_BYTES))
        self._sample_num = (self._sample_num + 1) % 256
        return bytes(packet)

    @property
    def in_waiting(self):
        if len(self._buffer) < 33:
            self._buffer.extend(self._generate_packet())
        return len(self._buffer)

    def read(self, n=1):
        time.sleep(n / (250 * 33))  # realistic byte timing at 250 Hz
        data = bytes(self._buffer[:n])
        del self._buffer[:n]
        return data

    def write(self, data):
        pass  # ignore start/stop commands

    def close(self):
        self.is_open = False

ser = SimulatedSerial()

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
                    print("Valid filtered packet:", filtered)
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




print("EEG1: ", EEG1)
print("EEG2: ", EEG2)
print("EEG3: ", EEG3)
print("EEG4: ", EEG4)
print("EEG5: ", EEG5)
print("EEG6: ", EEG6)
print("EEG7: ", EEG7)
print("EEG8: ", EEG8)