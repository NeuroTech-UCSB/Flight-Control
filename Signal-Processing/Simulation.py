import random
import struct
import time
import collections
import numpy as np
import joblib
import serial
from serial.tools import list_ports
from EEGRawFilter import filter_sample_live

WARMUP_PACKETS = 10
START_BYTE = 0xA0
STOP_BYTES = [0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF]

# ── Classifier config ──────────────────────────────────────────────────────────
CLF_PATH       = "clf.pkl"
FS             = 250
WIN_SAMPLES    = int(0.25 * FS)   # 62 — matches notebook win_s = 0.25
STRIDE_SAMPLES = int(0.125 * FS)  # 31 — matches notebook stride_s = 0.125
CLENCH_THRESH  = 0.5

clf = joblib.load(CLF_PATH)
window_buffer  = collections.deque(maxlen=WIN_SAMPLES)
stride_counter = 0
clench_event_count = 0
# ──────────────────────────────────────────────────────────────────────────────

# ── Simulation noise/clench config ────────────────────────────────────────────
# SCALE_UV = 0.02235 µV/count, so:
#   resting EEG ~±50 µV  →  std ~2 237 counts
#   jaw clench  ~±900 µV →  std ~40 000 counts
NOISE_STD        = 2_237    # resting EEG noise in raw counts
CLENCH_STD       = 40_000   # jaw clench burst amplitude in raw counts
CLENCH_INTERVAL  = 5 * FS   # trigger a clench every 5 seconds (1250 samples)
CLENCH_DURATION  = int(0.3 * FS)  # clench lasts 300 ms (75 samples)
# ──────────────────────────────────────────────────────────────────────────────

def int24_to_bytes(value):
    if value < 0:
        value += 1 << 24
    return value.to_bytes(3, byteorder='big')

class SimulatedSerial:
    def __init__(self):
        self._buffer      = bytearray()
        self._sample_num  = 0
        self._total_count = 0
        self.is_open      = True

    def _generate_packet(self):
        # Determine if this sample falls inside a clench burst
        phase = self._total_count % CLENCH_INTERVAL
        clenching = phase < CLENCH_DURATION
        std = CLENCH_STD if clenching else NOISE_STD

        packet = bytearray()
        packet.append(START_BYTE)
        packet.append(self._sample_num & 0xFF)
        for _ in range(8):
            eeg_value = int(np.random.normal(0, std))
            eeg_value = max(-(2**23), min(2**23 - 1, eeg_value))  # clamp to 24-bit
            packet.extend(int24_to_bytes(eeg_value))
        for _ in range(3):
            aux_value = random.randint(-32768, 32767)
            packet.extend(struct.pack('>h', aux_value))
        packet.append(random.choice(STOP_BYTES))
        self._sample_num  = (self._sample_num + 1) % 256
        self._total_count += 1
        return bytes(packet)

    @property
    def in_waiting(self):
        if len(self._buffer) < 33:
            self._buffer.extend(self._generate_packet())
        return len(self._buffer)

    def read(self, n=1):
        time.sleep(n / (250 * 33))
        data = bytes(self._buffer[:n])
        del self._buffer[:n]
        return data

    def write(self, data):
        pass

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
    if not chunk:
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
                    print("Valid filtered packet:", filtered)

                    # ── Windowed classifier ────────────────────────────────────
                    # filtered is [ch0..ch7] — append as one (8,) row
                    window_buffer.append(filtered)
                    stride_counter += 1

                    # Only predict once buffer is full AND a full stride has passed
                    if len(window_buffer) == WIN_SAMPLES and stride_counter >= STRIDE_SAMPLES:
                        stride_counter = 0

                        # Shape: (62, 8) — axis=0 is time, axis=1 is channel
                        # Matches notebook: X_windows[i] has shape (W, 8)
                        Xw = np.array(window_buffer)             # (62, 8)

                        # Features — identical to notebook Cell 3:
                        #   rms = np.sqrt(np.mean(X_windows**2, axis=1)) → per-window time axis
                        #   for a single window, time is axis=0
                        rms = np.sqrt(np.mean(Xw ** 2, axis=0))  # (8,)
                        var = np.var(Xw, axis=0)                  # (8,)
                        f   = np.concatenate([rms, var]).reshape(1, -1)  # (1, 16)

                        prob  = clf.predict_proba(f)[0, 1]
                        t_now = packets_received / FS

                        if prob >= CLENCH_THRESH:
                            clench_event_count += 1
                            print(f"\n{'!'*50}")
                            print(f"  JAW CLENCH #{clench_event_count}  |  t={t_now:.2f}s  |  prob={prob:.3f}")
                            print(f"{'!'*50}\n")
                        else:
                            print(f"  t={t_now:.2f}s  no clench  (prob={prob:.3f})")
                    # ──────────────────────────────────────────────────────────

                packets_received += 1
                if packets_received >= 2000:  # ~8s of data — enough for multiple clench windows
                    ser.write(b"s")
                    ser.close()
                    break
            else:
                buffer.pop(0)
        else:
            break

print("Num of Valid Packets: ", len(validPackets))
print("EEG1: ", EEG1)
print("EEG2: ", EEG2)
print("EEG3: ", EEG3)
print("EEG4: ", EEG4)
print("EEG5: ", EEG5)
print("EEG6: ", EEG6)
print("EEG7: ", EEG7)
print("EEG8: ", EEG8)
