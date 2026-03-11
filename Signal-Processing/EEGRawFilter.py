from scipy.signal import butter, iirnotch, sosfilt_zi, sosfilt, lfilter_zi, lfilter

SAMPLE_RATE = 250  # Hz

def make_bandpass(lowcut=1.0, highcut=50.0, fs=SAMPLE_RATE, order=4):
    sos = butter(order, [lowcut, highcut], btype='band', fs=fs, output='sos')
    return sos

def make_notch(freq=60.0, Q=30.0, fs=SAMPLE_RATE):
    b, a = iirnotch(freq, Q, fs)
    return b, a

# Build once, reuse
bandpass_sos = make_bandpass()
notch_b, notch_a = make_notch()
# Initialize filter states once
bp_zi  = sosfilt_zi(bandpass_sos)                    # shape: (n_sections, 2)
notch_zi = lfilter_zi(notch_b, notch_a)

# Scale factors per channel state — one zi per channel
bp_states    = [bp_zi.copy()   for _ in range(8)]
notch_states = [notch_zi.copy() for _ in range(8)]

def filter_sample_live(sample_uv: float, ch: int):
    # Bandpass
    out, bp_states[ch] = sosfilt(bandpass_sos, [sample_uv], zi=bp_states[ch])
    # Notch
    out, notch_states[ch] = lfilter(notch_b, notch_a, out, zi=notch_states[ch])
    return float(out[0])