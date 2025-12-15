#!/usr/bin/env python3
import smbus2
import time
import csv
import sys
from datetime import datetime
from adxl345 import ADXL, OutputDataRate

def init_adxl(adxl):
    """Initialize ADXL345 for continuous measurement with FIFO."""
    print("\n=== ADXL345 INITIALIZATION ===")

    # Read device ID to verify communication
    devid = adxl.bus.read_byte_data(adxl.address, 0x00)
    print(f"Device ID: 0x{devid:02X} (expected 0xE5)")
    if devid != 0xE5:
        raise RuntimeError(f"Invalid device ID: 0x{devid:02X}")

    # Set FULL_RES mode (±16g range with full resolution)
    print("Setting FULL_RES mode (±16g)...")
    adxl.data_format.write("FULL_RES", 1)


    # Set FIFO to STREAM mode
    adxl.fifo_ctl.write("MODE", 0b10)  # STREAM mode

    # Enable measurement mode
    print("Enabling MEASUREMENT mode...")
    adxl.power_control.write("MEASURE", 1)

    # Verify settings
    print("\n*** REGISTER VERIFICATION ***")
    print(f"POWER_CTL.MEASURE:    {adxl.power_control.read('MEASURE')}")
    print(f"DATA_FORMAT.FULL_RES: {adxl.data_format.read('FULL_RES')}")
    print(f"BW_RATE.RATE:         0x{adxl.bandwidth_rate.read('RATE'):02X}")
    print(f"FIFO_CTL.MODE:        0b{adxl.fifo_ctl.read('MODE'):02b}")
    print(f"FIFO_CTL.SAMPLES:     {adxl.fifo_ctl.read('SAMPLES')}")
    print(f"FIFO_STATUS.ENTRIES:  {adxl.fifo_status.read('ENTRIES')}")

    print("\nInitialization complete.\n")


def draw_fifo_bar(num_entries, watermark=28, max_size=32, bar_width=40):
    """Draw a visual representation of FIFO fill level.

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

    print(f"=== STARTING CONTINUOUS ACQUISITION ===")
    print(f"Duration: {duration_seconds}s")
    print(f"Sample rate: {sample_rate} Hz")
    print(f"Expected samples: ~{duration_seconds * sample_rate}")
    print(f"Watermark set at {adxl.watermark} samples")
    print(f"Reading when samples available...\n")

    overflow_count = 0
    read_count = 0
    samples = []
    last_watermark = False

    # Print initial status line
    print("FIFO Status:")
    watermark_flag = adxl.interrupt_source.read("WATERMARK")
    time.sleep(0.05)

    start_time = time.time()
    while time.time() - start_time < duration_seconds:
        # Check FIFO status
        num_entries = adxl.fifo_status.read("ENTRIES")
        watermark_flag = adxl.interrupt_source.read("WATERMARK")
        fifo_overflow = adxl.interrupt_source.read("OVERRUN")

        # Draw FIFO bar (update in place)
        fifo_bar = draw_fifo_bar(num_entries, watermark=adxl.watermark)
        watermark_indicator = "!" if watermark_flag else "  "
        overflow_indicator = fifo_overflow and num_entries > adxl.watermark


        # Check for overflow
        if overflow_indicator:
            # Print on same line using carriage return
            sys.stdout.write(f"\r{fifo_bar} {watermark_indicator} {overflow_indicator} Samples: {len(samples):4d}")
            sys.stdout.flush()
            overflow_count += 1
            print(f"\n WARNING: FIFO overflow detected! (count: {overflow_count})")
        else:
            # Print on same line using carriage return
            sys.stdout.write(f"\r{fifo_bar} Samples: {len(samples):4d}")
            sys.stdout.flush()

        # Watermark status change notification
        if watermark_flag and not last_watermark:
            print(f"\n Watermark reached at {len(samples)} samples")
        last_watermark = watermark_flag

        # Read samples if available
        if num_entries > 10:
            read_count += 1
            samples.append(adxl.get_accel())
        else:
            # Give more time for FIFO to fill up
            time.sleep(0.01)
            continue

        # Small sleep to avoid hammering I2C bus
        time.sleep(0.001)

    # Move to new line after progress bar
    print("\n")

    # Create timestamped samples
    timestamped_samples = []
    for i, (x, y, z) in enumerate(samples):
        timestamp = i * sample_period
        timestamped_samples.append((timestamp, x, y, z))

    print(f"=== ACQUISITION COMPLETE ===")
    print(f"Collected {len(timestamped_samples)} samples in {duration_seconds}s")
    print(f"Expected: ~{duration_seconds * sample_rate}")
    data_loss = max(0, duration_seconds * sample_rate - len(timestamped_samples))
    print(f"Data loss: {data_loss} samples ({data_loss / (duration_seconds * sample_rate) * 100:.2f}%)")
    print(f"Overflow events: {overflow_count}")
    print(f"Read operations: {read_count}\n")

    return timestamped_samples


def write_to_csv(samples, filename=None):
    """Write samples with timestamps to CSV file.

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
        print()

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

def main():
    """Main measurement routine."""
    # Initialize I2C bus and ADXL345
    bus = smbus2.SMBus(1)
    adxl = ADXL(
        0x1D,
        bus,
        watermark=28,
        odr=OutputDataRate.ODR_200
    )

    # Initialize the device
    init_adxl(adxl)

    # Acquisition parameters
    duration = 10  # seconds

    # Perform continuous acquisition
    samples = read_continuous(adxl, duration_seconds=duration)

    # Write to CSV
    filename = write_to_csv(samples)

    # Print preview
    print_sample_preview(samples, num_preview=10)

    print("=" * 57)
    print(f"Measurement complete!")
    print(f"Data saved to: {filename}")
    print("=" * 57)
    print()

    terminate(adxl)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nMeasurement interrupted by user.")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
