import pandas as pd
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

# Read the data
df = pd.read_csv("data/accelerometer_data_20251209_162505.csv")

# Use time_relative for uniform sampling
t = df['time_s'].values[:200]
x = df['x_g'].values[:200]

# Removing bias
x = x - np.mean(x)


# FFT
fft = np.fft.fft(x)
freq = np.fft.fftfreq(len(x), d=0.01)  # d = 1/sample_rate

# Welch PSD
f, psd = signal.welch(x, fs=100, nperseg=256)

# Plot
plt.figure(figsize=(12, 4))
plt.subplot(211)
plt.plot(t, x)
plt.title("STS - Cold Head Off")
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (g)')

plt.subplot(212)
plt.semilogy(f, psd)
plt.xlabel('Frequency (Hz)')
plt.ylabel('PSD')
plt.savefig("ColdHeadOn.png")
plt.show()
