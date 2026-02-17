#!/usr/bin/env python3
import smbus2
from adxl345 import ADXL345, OutputDataRate, Range  # adjust import if needed
import measure



def select_enum(enum_cls, default):
    print(
        f" Select {enum_cls.__name__} (press Enter for default: {default.name})"
    )

    members = list(enum_cls)
    for i, member in enumerate(members):
        print(f"{i}: {member.display}")


    choice = input("> ").strip()

    if choice == "":
        return default

    try:
        return members[int(choice)]
    except (ValueError, IndexError):
        print("Invalid selection. Using default.")
        return default


def confirm_settings(odr, g_range):
    print("\nSelected settings:")
    print(f"ODR   : {odr.name} ({odr.hz} Hz)")
    print(f"Range : {g_range.name}")

    confirm = input("Start measurement? (y/n): ").strip().lower()
    return confirm == "y"


def main():
    BUS_NUM = 1
    DEVICE_ADDR = 0x53  # typical ADXL345 address

    default_odr = OutputDataRate.ODR_100
    default_range = Range.RANGE_FULL

    odr = select_enum(OutputDataRate, default_odr)
    g_range = select_enum(Range, default_range)

    if not confirm_settings(odr, g_range):
        print("Startup aborted.")
        return

    bus = smbus2.SMBus(BUS_NUM)
    sensor = ADXL345(
        address=DEVICE_ADDR,
        bus=bus,
        odr=odr,
        g_range=g_range
    )

    measure.measure(sensor)


if __name__ == "__main__":
    main()

