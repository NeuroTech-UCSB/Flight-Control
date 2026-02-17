import random
import struct
import time
import sys

START_BYTE = 0xA0
STOP_BYTES = [0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6] 

def int24_to_bytes(value):
    """
    Convert signed 24-bit integer to 3 bytes (big-endian)
    """
    if value < 0:
        value += 1 << 24
    return value.to_bytes(3, byteorder='big')

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


stream = cyton_byte_stream(max_packets=100)

buffer = bytearray()

validPackets = []

while True:
    try:
        byte = next(stream)
    except StopIteration:
        break
    buffer.append(byte)
    
    if len(buffer) >= 33:
        if buffer[0] == 0xA0 and buffer[32] in STOP_BYTES:
            packet = buffer[:33]
            if len(packet) != 33:
                continue
            buffer = buffer[33:]
            validPackets.append(packet)
            print("Valid packet:", packet.hex())
        else:
            try:
                next_start = buffer.index(START_BYTE, 1)
                buffer = buffer[next_start:]
            except ValueError:
                buffer.clear()

print("Num of Valid Packets: ",  len(validPackets))
