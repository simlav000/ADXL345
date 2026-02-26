import pandas as pd
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt

# Read the data
df = pd.read_csv(
    "/home/simlav000/McGill/STS/Accelerometer/data/accelerometer_data_20260226_133020.csv",
    skiprows=1
)
# Use time_relative for uniform sampling
t = df['time_s'].values#[:200]
x = df['x_g'].values#[:200]
y = df['y_g'].values#[:200]
z = df['z_g'].values#[:200]

acc_mag = np.sqrt(x**2 + y**2 + z**2)

# Remove bias (e.g. gravity)
acc_mag -= np.mean(acc_mag)


# FFT
fft = np.fft.fft(acc_mag)
freq = np.fft.fftfreq(len(acc_mag), d=0.01)  # d = 1/sample_rate

# Welch PSD
f, psd = signal.welch(acc_mag, fs=100, nperseg=256)

# Plot
plt.figure(figsize=(12, 4))
plt.subplot(211)
plt.plot(t, acc_mag)
plt.title("STS - Cold Head Off")
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (g)')

plt.subplot(212)
plt.semilogy(f, psd)
plt.xlabel('Frequency (Hz)')
plt.ylabel('PSD')
plt.savefig("ColdHeadOn.png")
plt.show()
