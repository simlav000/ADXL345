#!/usr/bin/env python3
import smbus2
import time
import csv
from datetime import datetime
from adxl345 import ADXL

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

    # Set bandwidth rate to 100 Hz
    print("Setting data rate to 100 Hz...")
    adxl.bandwidth_rate.write("RATE", 0x0A)  # 0x0A = 100 Hz

    # Clear FIFO by setting to BYPASS mode, then back to STREAM
    print("Clearing FIFO...")
    adxl.fifo_ctl.write("MODE", 0b00)  # BYPASS mode
    time.sleep(0.01)

    # Set FIFO to STREAM mode with watermark at 28
    print("Setting FIFO to STREAM mode with watermark at 28...")
    adxl.fifo_ctl.write("MODE", 0b10)  # STREAM mode
    adxl.fifo_ctl.write("SAMPLES", 28)  # Watermark at 28 samples

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


def read_continuous(adxl, duration_seconds=10, sample_rate=100):
    """Continuously read FIFO with watermark monitoring to avoid data loss.

    Args:
        adxl: ADXL instance
        duration_seconds: How long to acquire data
        sample_rate: Expected sample rate in Hz

    Returns:
        list: List of (timestamp, x_g, y_g, z_g) tuples
    """
    all_samples = []
    start_time = time.time()
    sample_period = 1.0 / sample_rate

    print(f"=== STARTING CONTINUOUS ACQUISITION ===")
    print(f"Duration: {duration_seconds}s")
    print(f"Sample rate: {sample_rate} Hz")
    print(f"Expected samples: ~{duration_seconds * sample_rate}")
    print(f"Watermark set at 28 samples")
    print(f"Reading when watermark is reached...\n")

    overflow_count = 0
    read_count = 0

    while time.time() - start_time < duration_seconds:
        # Check FIFO status
        num_entries = adxl.fifo_status.read("ENTRIES")
        watermark_flag = adxl.interrupt_source.read("WATERMARK")
        fifo_overflow = adxl.interrupt_source.read("OVERRUN")


        if fifo_overflow:
            overflow_count += 1
            print(f"WARNING: FIFO overflow detected! (count: {overflow_count})")

        # Read when watermark is reached or FIFO is getting full
        if watermark_flag or num_entries >= 28:
            read_count += 1
            print(f"Read #{read_count}: {num_entries} samples in FIFO (watermark: {watermark_flag})")

            # Read all available samples
            for i in range(num_entries):
                x_g, y_g, z_g = adxl.get_accel()

                # Calculate timestamp assuming uniform sampling
                timestamp = len(all_samples) * sample_period
                all_samples.append((timestamp, x_g, y_g, z_g))

            print(f"  → Total samples collected: {len(all_samples)}")

        # Small sleep to avoid hammering I2C bus
        time.sleep(0.01)

    print(f"\n=== ACQUISITION COMPLETE ===")
    print(f"Collected {len(all_samples)} samples in {duration_seconds}s")
    print(f"Expected: ~{duration_seconds * sample_rate}")
    data_loss = max(0, duration_seconds * sample_rate - len(all_samples))
    print(f"Data loss: {data_loss} samples ({data_loss / (duration_seconds * sample_rate) * 100:.2f}%)")
    print(f"Overflow events: {overflow_count}")
    print(f"Read operations: {read_count}\n")

    return all_samples


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
        filename = f"accelerometer_data_{timestamp}.csv"

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

    print(f"✓ Successfully wrote data to {filename}\n")
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


def main():
    """Main measurement routine."""
    # Initialize I2C bus and ADXL345
    bus = smbus2.SMBus(1)
    adxl = ADXL(0x1D, bus)

    # Initialize the device
    init_adxl(adxl)

    # Acquisition parameters
    duration = 10  # seconds
    sample_rate = 100  # Hz

    # Perform continuous acquisition
    samples = read_continuous(adxl, duration_seconds=duration, sample_rate=sample_rate)

    # Write to CSV
    filename = write_to_csv(samples)

    # Print preview
    print_sample_preview(samples, num_preview=10)

    print("=" * 50)
    print(f"✓ Measurement complete!")
    print(f"✓ Data saved to: {filename}")
    print("=" * 50)
    print()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nMeasurement interrupted by user.")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
