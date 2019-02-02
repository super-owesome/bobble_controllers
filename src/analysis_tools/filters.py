import scipy


def butter_bandpass(lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = scipy.signal.butter(order, [low, high], btype='band')
    return b, a


def butter_bandstop(lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = scipy.signal.butter(order, [low, high], btype='bandstop')
    return b, a


def butter_bandpass_filter(data, lowcut, highcut, fs, order=5):
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    y = scipy.signal.lfilter(b, a, data)
    return y


def butter_bandstop_filter(data, lowcut, highcut, fs, order=5):
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    i, u = scipy.signal.butter(order, [low, high], btype='bandstop')
    y = scipy.signal.lfilter(i, u, data)
    return y

def butter_lowpass(cutoff_freq, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff_freq / nyq
    b, a = scipy.signal.butter(order, normal_cutoff, btype='low', analog=False)
    return b, a


def butter_lowpass_filter(data, sampling_rate, cutoff_freq, order=5):
    b, a = butter_lowpass(cutoff_freq, sampling_rate, order=order)
    y = scipy.signal.lfilter(b, a, data)
    return y
