import pandas as pd
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
from pathlib import Path

fname = Path("data/accelerometer_data_20260302_123634.csv")

# Read first row only (metadata row)
meta_row = pd.read_csv(fname, nrows=1, header=None).iloc[0]

is_stationary =  meta_row[0]
odr = meta_row[1]
range_g = meta_row[2]

df = pd.read_csv(fname, skiprows=1)

# Use time_relative for uniform sampling
t = df['time_s'].values[100:]
x = df['x_g'].values[100:]
y = df['y_g'].values[100:]
z = df['z_g'].values[100:]

mag = np.sqrt(x**2 + y**2 + z**2)
mag -= np.mean(mag)

# FFT
fft = np.fft.fft(mag)
freq = np.fft.fftfreq(len(mag), d=0.01)  # d = 1/sample_rate

# Welch PSD
f, psd = signal.welch(mag, fs=100, nperseg=256)

# Output filename (same directory, same stem, different suffix)
output_path = fname.with_suffix(".png")

# Welch PSD
f, psd = signal.welch(acc_mag, fs=100, nperseg=256)

# Plot
plt.figure(figsize=(12, 4))

plt.subplot(211)
plt.plot(t, mag)
plt.title(f"{fname.stem} | {is_stationary} | {odr} | {range_g}")
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (g)')

plt.subplot(212)
plt.plot(f, psd)
plt.xlabel('Frequency (Hz)')
plt.ylabel('PSD')

plt.tight_layout()
plt.savefig(output_path)
plt.show()
