from scipy.signal import butter, iirnotch, sosfilt_zi, sosfilt, lfilter_zi, lfilter, welch


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

# Scale factors per channe  l state — one zi per channel
bp_states    = [bp_zi.copy()   for _ in range(8)]
notch_states = [notch_zi.copy() for _ in range(8)]

def filter_sample_live(sample_uv: float, ch: int):
    # Bandpass
    out, bp_states[ch] = sosfilt(bandpass_sos, [sample_uv], zi=bp_states[ch])
    # Notch
    out, notch_states[ch] = lfilter(notch_b, notch_a, out, zi=notch_states[ch])
    return float(out[0])

def convert_to_power_spectral_density(filtered_samples_uv):
    f, Pxx = welch(filtered_samples_uv, fs=SAMPLE_RATE, window='hann', nperseg=256, noverlap=128)
    return f, Pxx

def extract_band_powers(Xw):
    """
    Compute per-band power for each of 8 channels from a window of EEG samples.

    Parameters:
        Xw: np.ndarray of shape (n_samples, 8) — filtered EEG window

    Returns:
        theta : np.ndarray (8,) — mean power 4–8 Hz per channel
        alpha : np.ndarray (8,) — mean power 8–13 Hz per channel
        beta  : np.ndarray (8,) — mean power 13–30 Hz per channel
    """
    import numpy as np
    n_samples = Xw.shape[0]
    nperseg = min(256, n_samples)

    theta = np.zeros(8)
    alpha = np.zeros(8)
    beta  = np.zeros(8)

    for ch in range(8):
        f, Pxx = welch(Xw[:, ch], fs=SAMPLE_RATE, window='hann', nperseg=nperseg, noverlap=nperseg // 2)
        theta[ch] = Pxx[(f >= 4)  & (f < 8)].mean()
        alpha[ch] = Pxx[(f >= 8)  & (f < 13)].mean()
        beta[ch]  = Pxx[(f >= 13) & (f < 30)].mean()

    return theta, alpha, beta