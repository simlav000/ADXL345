#!/usr/bin/env python3
import smbus2
import time
import csv
import sys
from datetime import datetime
from adxl345 import ADXL345, OutputDataRate

def init_adxl(adxl):
    """Initialize ADXL345 for continuous measurement with FIFO."""
    print("\n=== ADXL345 INITIALIZATION ===")

    # Set FIFO to STREAM mode
    adxl.fifo_ctl.write("MODE", 0b10)  # STREAM mode

    # Enable measurement mode
    print("Enabling MEASUREMENT mode...")
    adxl.power_control.write("MEASURE", 1)

    # Verify settings
    print("\n*** REGISTER VERIFICATION ***")
    print(f"POWER_CTL.MEASURE:    {adxl.power_control.read('MEASURE')}")
    print(f"BW_RATE.RATE:         0x{adxl.bandwidth_rate.read('RATE'):02X}")
    print(f"FIFO_CTL.MODE:        0b{adxl.fifo_ctl.read('MODE'):02b}")
    print(f"FIFO_CTL.SAMPLES:     {adxl.fifo_ctl.read('SAMPLES')}")
    print(f"FIFO_STATUS.ENTRIES:  {adxl.fifo_status.read('ENTRIES')}")

    print("\nInitialization complete.\n")


def draw_fifo_bar(num_entries, watermark=28, max_size=32, bar_width=40):
    """Draw a visual representation of FIFO fill level.
    This is SLOW. Was using it for fun but you can't achieve very fast
    ODRs without overflow here.

    Args:
        num_entries: Current number of samples in FIFO
        watermark: Watermark level
        max_size: Maximum FIFO size
        bar_width: Width of the bar in characters

    Returns:
        str: Visual bar representation
    """
    # Calculate how many characters to fill
    fill_chars = int((num_entries / max_size) * bar_width)
    watermark_pos = int((watermark / max_size) * bar_width)

    # Build the bar
    bar = ""
    for i in range(bar_width):
        if i < fill_chars:
            bar += "█"
        elif i == watermark_pos:
            bar += "|"  # Show watermark position
        else:
            bar += "░"

    return f"[{bar}] {num_entries:2d}/32"


def read_continuous(adxl, duration_seconds=10):
    """Continuously read FIFO with watermark monitoring to avoid data loss.

    Args:
        adxl: ADXL instance
        duration_seconds: How long to acquire data
        sample_rate: Expected sample rate in Hz

    Returns:
        list: List of (timestamp, x_g, y_g, z_g) tuples
    """
    sample_rate = adxl.odr.hz
    sample_period = 1.0 / sample_rate
    watermark = adxl.watermark

    print(f"=== STARTING CONTINUOUS ACQUISITION ===")
    print(f"Duration: {duration_seconds}s")
    print(f"Sample rate: {sample_rate} Hz")
    print(f"Expected samples: ~{duration_seconds * sample_rate}")
    print(f"Watermark set at {adxl.watermark} samples")
    print(f"Reading when samples available...\n")

    overflow_count = 0
    read_count = 0
    samples = []

    # Read INT_SOURCE register to clear overrun and watermark bits
    watermark_flag = adxl.interrupt_source.read("WATERMARK")
    time.sleep(0.01)

    start_time = time.time()

    while time.time() - start_time < duration_seconds:
        watermark_reached = adxl.interrupt_source.read("WATERMARK")
        overflow_occured = adxl.interrupt_source.read("OVERRUN")

        if overflow_occured:
            overflow_count += 1

        # Read waterflow amount of samples
        if watermark_reached:
            for _ in range(watermark):
                samples.append(adxl.get_accel())
            read_count += watermark

    # Create timestamped samples
    timestamped_samples = []
    for i, (x, y, z) in enumerate(samples):
        timestamp = i * sample_period
        timestamped_samples.append((timestamp, x, y, z))

    print(f"=== ACQUISITION COMPLETE ===")
    print(f"Collected {len(timestamped_samples)} samples in {duration_seconds}s")
    print(f"Expected: ~{duration_seconds * sample_rate}")
    print(f"Overflow events: {overflow_count}")
    print(f"Read operations: {read_count}\n")

    return timestamped_samples


def write_to_csv(samples, settings, filename=None):
    """Write samples with timestamps and metadata to CSV file.

    Args:
        samples: List of (timestamp, x_g, y_g, z_g) tuples
        filename: Output filename (auto-generated if None)

    Returns:
        str: The filename written to
    """
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = fr"data/accelerometer_data_{timestamp}.csv"

    print(f"Writing {len(samples)} samples to {filename}...")

    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)

        # Write metadata (settings)
        writer.writerow(settings)
        # Write header
        writer.writerow(['time_s', 'x_g', 'y_g', 'z_g'])

        # Write data
        for timestamp, x_g, y_g, z_g in samples:
            writer.writerow([
                f"{timestamp:.6f}",
                f"{x_g:.6f}",
                f"{y_g:.6f}",
                f"{z_g:.6f}"
            ])

    return filename


def print_sample_preview(samples, num_preview=10):
    """Print first and last few samples as preview."""
    if not samples:
        print("No samples to preview.")
        return

    print("=== SAMPLE PREVIEW ===")
    print(f"{'Time(s)':<12} {'X(g)':<10} {'Y(g)':<10} {'Z(g)':<10}")
    print("-" * 42)

    # First samples
    for timestamp, x, y, z in samples[:num_preview]:
        print(f"{timestamp:<12.6f} {x:<10.4f} {y:<10.4f} {z:<10.4f}")

    if len(samples) > num_preview * 2:
        print("...")

        # Last samples
        for timestamp, x, y, z in samples[-num_preview:]:
            print(f"{timestamp:<12.6f} {x:<10.4f} {y:<10.4f} {z:<10.4f}")

    print()

def flush(adxl):
    adxl.fifo_ctl.write("MODE", 0b00)  # BYPASS mode
    num_entries = adxl.fifo_status.read("ENTRIES")
    while num_entries != 0:
        adxl.get_accel()

def terminate(adxl):
    # Sets sensor to BYPASS mode so FIFO does not fill
    adxl.fifo_ctl.write("MODE", 0b00)  # BYPASS mode

def measure(sensor: ADXL345, is_stationary : bool = False):
    """
    Main measurement routine.
        sensor: ADXL345 object allowing register operations.
        is_stationary: Whether this is an active or passive run.
    """
    init_adxl(sensor)

    # Acquisition parameters
    duration = 10 # seconds

    settings = [
        f"Stationary: {is_stationary}",
        f"Output Data Rate:{sensor.odr.hz} Hz ",
        f"Range: {sensor.g_range.g} g"
    ]

    # Perform continuous acquisition
    samples = read_continuous(sensor, duration_seconds=duration)

    # Write to CSV
    filename = write_to_csv(samples, settings)

    # Print preview
    print_sample_preview(samples, num_preview=10)

    print("=" * 57)
    print(f"Measurement complete!")
    print(f"Data saved to: {filename}")
    print("=" * 57)
    print()

    terminate(sensor)


if __name__ == "__main__":
    try:
        bus = smbus2.SMBus(1)
        sensor = ADXL345(
            0x53,
            bus,
            watermark=28,
            odr=OutputDataRate.ODR_100
        )

        measure(sensor)
    except KeyboardInterrupt:
        print("\n\nMeasurement interrupted by user.")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
