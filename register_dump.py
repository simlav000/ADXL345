#!/usr/bin/env python3

from adxl345 import ADXL345

def decode_fifo_mode(mode_bits):
    modes = {
        0b00: "Bypass",
        0b01: "FIFO",
        0b10: "Stream",
        0b11: "Trigger"
    }
    return modes.get(mode_bits, "Unknown")

def main():
    sensor = ADXL345()

    print("\n--- ADXL345 REGISTER DUMP ---\n")

    # Raw register reads
    power_ctl = sensor.bus.read_byte_data(sensor.address, 0x2D)
    data_format = sensor.bus.read_byte_data(sensor.address, 0x31)
    bw_rate = sensor.bus.read_byte_data(sensor.address, 0x2C)
    fifo_ctl = sensor.bus.read_byte_data(sensor.address, 0x38)
    fifo_status = sensor.bus.read_byte_data(sensor.address, 0x39)

    print(f"POWER_CTL   (0x2D): 0b{power_ctl:08b}")
    print(f"DATA_FORMAT (0x31): 0b{data_format:08b}")
    print(f"BW_RATE     (0x2C): 0b{bw_rate:08b}")
    print(f"FIFO_CTL    (0x38): 0b{fifo_ctl:08b}")
    print(f"FIFO_STATUS (0x39): 0b{fifo_status:08b}")
    print()

    # --- Decode DATA_FORMAT ---
    full_res = (data_format >> 3) & 0x01
    range_bits = data_format & 0x03

    ranges = {
        0b00: "±2g",
        0b01: "±4g",
        0b10: "±8g",
        0b11: "±16g"
    }

    print("DATA_FORMAT decoded:")
    print(f"  FULL_RES: {full_res}")
    print(f"  Range:    {ranges.get(range_bits)}")
    print()

    # --- Decode BW_RATE ---
    low_power = (bw_rate >> 4) & 0x01
    rate_code = bw_rate & 0x0F

    print("BW_RATE decoded:")
    print(f"  LOW_POWER: {low_power}")
    print(f"  Rate code: {rate_code}")
    print()

    # --- Decode FIFO_CTL ---
    fifo_mode = (fifo_ctl >> 6) & 0x03
    trigger_bit = (fifo_ctl >> 5) & 0x01
    samples = fifo_ctl & 0x1F

    print("FIFO_CTL decoded:")
    print(f"  Mode:          {decode_fifo_mode(fifo_mode)}")
    print(f"  Trigger bit:   {trigger_bit}")
    print(f"  Samples level: {samples}")
    print()

    # --- Decode FIFO_STATUS ---
    entries = fifo_status & 0x3F
    fifo_trigger = (fifo_status >> 7) & 0x01

    print("FIFO_STATUS decoded:")
    print(f"  Entries in FIFO: {entries}")
    print(f"  Triggered:       {fifo_trigger}")
    print()

    print("--- END DUMP ---\n")


if __name__ == "__main__":
    main()
