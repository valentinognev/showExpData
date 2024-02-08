import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft

def lowpass_filter(signal, cutoff_frequency, sampling_rate):
    """
    Apply a simple low-pass filter to the input signal.

    Parameters:
    - signal: The input signal.
    - cutoff_frequency: The cutoff frequency for the low-pass filter.
    - sampling_rate: The sampling rate of the signal.

    Returns:
    - filtered_signal: The filtered output signal.
    """

    # Calculate filter coefficients
    alpha = 2 * np.pi * cutoff_frequency / sampling_rate
    
    # Initialize variables
    filtered_signal = np.zeros_like(signal)
    prev_output = 0.0

    # Apply the low-pass filter
    for i in range(len(signal)):
        current_input = signal[i]
        current_output = alpha * current_input + (1-alpha)*prev_output
        filtered_signal[i] = current_output
        prev_output = current_output

    return filtered_signal

# Plot FFT of the original and filtered signals
def plot_fft(signal, title):
    spectrum = fft(signal)
    freq = np.fft.fftfreq(len(signal), d=1/sampRate)
    plt.figure(1,figsize=(10, 6))
    plt.plot(freq, np.abs(spectrum))
    plt.title(title)
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Amplitude')
    plt.grid(True)

# Example usage:
# Set your sampling rate (sampRate), cutoff frequency, and generate example data
sampRate = 1000  # replace with your actual sampling rate
cutoff_frequency = 200  # replace with your desired cutoff frequency

# Generate a uniform random signal
np.random.seed(42)  # for reproducibility
time = np.arange(0, 10, 1/sampRate)
data = np.random.uniform(-1, 1, len(time))

# Apply low-pass filter
filtered_data = lowpass_filter(data, cutoff_frequency, sampRate)

# Plot FFT of the original and filtered signals
plot_fft(data, 'FFT of Original Signal')
plot_fft(filtered_data, 'FFT of Filtered Signal')


# Plot original and filtered data

plt.figure(2,figsize=(10, 6))
plt.plot(time, data, label='Original Data')
plt.plot(time, filtered_data, label='Filtered Data')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.legend()
plt.show()
a=1





