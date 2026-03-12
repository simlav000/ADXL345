import pandas as pd
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
from pathlib import Path


def load_metadata(fname):
    meta = pd.read_csv(fname, nrows=1, header=None).iloc[0]
    return meta.tolist()

def load_data(fname, start=0, stop=100):
    with open(fname) as f:
        first_line = f.readline()

    skip = 1 if "Range" in first_line else 0
    df = pd.read_csv(fname, skiprows=skip)
    print(f"Processing: {fname}")
    t_keys = ["time_s", "t_s", "time_relative"]

    for t_k in t_keys:
        try:
            t = df[t_k].values[start:]
            break
        except KeyError:
            continue
    else:
        raise KeyError("No valid time column found")

    x = df["x_g"].values[start:]
    y = df["y_g"].values[start:]
    z = df["z_g"].values[start:]

    return t, x, y, z


def compute_magnitude(x, y, z):
    mag = np.sqrt(x**2 + y**2 + z**2)
    mag -= np.mean(mag)
    return mag


def compute_psd(mag, fs=100):
    f, psd = signal.welch(mag, fs=fs, nperseg=256)
    return f, psd


def make_plot(t, mag, f, psd, meta, title, output_path, show=False):
    meta_str = " | ".join(str(m) for m in meta if str(m) != "nan")

    plt.figure(figsize=(12, 4))

    plt.subplot(211)
    plt.plot(t, mag)
    plt.title(f"{title} | {meta_str}")
    plt.xlabel("Time (s)")
    plt.ylabel("Acceleration (g)")

    plt.subplot(212)
    plt.plot(f, psd)
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("PSD")

    plt.tight_layout()
    plt.savefig(output_path)
    if show:
        print("show")
        plt.show()
    plt.close()

def process_file(fname, fig_dir):
    meta = load_metadata(fname)
    has_header = any("Range" in str(m) for m in meta)
    t, x, y, z = load_data(fname)

    mag = compute_magnitude(x, y, z)
    f, psd = compute_psd(mag)

    output = fig_dir / f"{fname.stem}.png"

    make_plot(t, mag, f, psd, meta, fname.stem, output, show=True)


def process_all(data_dir="data", fig_dir="Figures"):
    data_dir = Path(data_dir)
    fig_dir = Path(fig_dir)
    fig_dir.mkdir(exist_ok=True)

    for fname in data_dir.glob("*.csv"):
        process_file(fname, fig_dir)


if __name__ == "__main__":
    process_file(Path("data/accelerometer_data_20260305_112631.csv"), Path("Figures"))
    #process_all()
