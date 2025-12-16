#!/usr/bin/env python3
import smbus2
from enum import IntEnum

class OutputDataRate(IntEnum):
    """
    Enumerated class for allowable Output Data Rates (ODR) in Hz.
    12P5 means 12.5 Hz
    """
    ODR_3200 = 0b1111
    ODR_1600 = 0b1110
    ODR_800  = 0b1101
    ODR_400  = 0b1100
    ODR_200  = 0b1011
    ODR_100  = 0b1010 # Default
    ODR_50   = 0b1001
    ODR_25   = 0b1000
    ODR_12P5 = 0b0111
    ODR_6P25 = 0b0110
    ODR_3P13 = 0b0101
    ODR_1P56 = 0b0100
    ODR_0P78 = 0b0011
    ODR_0P39 = 0b0010
    ODR_0P20 = 0b0001
    ODR_0P10 = 0b0000

    @property
    def hz(self) -> float:
        # Get value in Hz from enum member's names
        return float(self.name[4:].replace("P", "."))


class Range(IntEnum):
    """
    Enumerated class for allowable acceleration range
    """
    RANGE_FULL = 0b1 # Default
    RANGE_16g  = 0b11
    RANGE_8g   = 0b10
    RANGE_4g   = 0b01
    RANGE_2g   = 0b00

    @property
    def g(self) -> int:
        # Get value in g's from enum member's names
        return int(self.name[6:-1])

class Register:
    """
    Register abstraction class

    Parameters
    ----------
    address: int
        Memory-mapped address of register (as per manual)
    read_only: bool
        Whether register can be written to
    fields: dict[str, int]
        Mapping between name of field and a bit mask that selects it.
        For example if the 6 right-most bits of a certain register delineates a
        field named "FIELD", it is selected by 0b0011_1111 or 0x3F
    """
    def __init__(self, address: int, read_only: bool, fields: dict[str, int]):
        self.device_address = None  # Set later
        self.bus = None  # Set later
        self.register_address = address
        self.fields = fields
        self.read_only = read_only

    def _bind(self, bus, device_address):
        """Called by ADXL to bind bus and device address."""
        self.bus = bus
        self.device_address = device_address

    def read(self, field: str):
        if self.bus is None:
            raise RuntimeError("Register not bound to a bus")

        mask = self.fields[field]
        shift_amount = (mask & -mask).bit_length() - 1
        reg_value = self.bus.read_byte_data(self.device_address, self.register_address)
        return (reg_value & mask) >> shift_amount

    def write(self, field: str, value: int):
        if self.read_only:
            raise RuntimeError("Cannot write to read-only register")
        if self.bus is None:
            raise RuntimeError("Register not bound to a bus")

        mask = self.fields[field]
        shift_amount = (mask & -mask).bit_length() - 1

        # Read-modify-write (to only modify selected field and keep other fields intact)
        reg_value = self.bus.read_byte_data(self.device_address, self.register_address)
        reg_value = (reg_value & ~mask) | ((value << shift_amount) & mask)
        self.bus.write_byte_data(self.device_address, self.register_address, reg_value)

class ADXL345:
    """
    ADXL345 interface class.

    Parameters:
    -----------
    address: int
        i2c address of device on your machine. Found by running `i2cdetect -y 1`
        if on raspberry pi.
    bus: smbus2.SMBus
        i2c bus from smbus2 package.
    odr: (Optional) OutputDataRate
        One of the provided output data rates for the sensor.
        Default: 100 Hz
    range: (Optional) Range
        One of the provided acceleration ranges for the sensor
        Default: Full resolution mode
    watermark: (Optional) int
        Number of values FIFO can hold before the watermark interrupt flag is
        set.
        Default: 28

    Ideally, one would implement methods to read each of the fields without
    having to use the "read" and "write" methods of the Register class, but
    I must move on.
    """
    def __init__(
            self,
            address: int,
            bus: smbus2.SMBus,
            odr = OutputDataRate.ODR_100,
            g_range = Range.RANGE_FULL,
            watermark: int = 28
    ):
        self.address = address # Found by running i2cdetect -y 1 on raspi
        self.bus = bus
        self.watermark = watermark
        self.odr = odr
        self.g_range = g_range

        # Define registers
        # WARNING: Registers 0x01 through 0x1C are reserved. Do not touch!
        self.bandwidth_rate = Register(0x2C, False, {
            "LOW_POWER"  : 0x10,
            "RATE"       : 0x0F
        })

        self.power_control = Register(0x2D, False, {
            "LINK"       : 0x20,
            "AUTO_SLEEP" : 0x10,
            "MEASURE"    : 0x08, # Setting to 0 places into standby mode, setting to 1 sets to measurement mode.
            "SLEEP"      : 0x04,
            "WAKEUP"     : 0x03
        })

        self.interrupt_source = Register(0x30, True, {
            "DATA_READY" : 0x80,
            "SINGLE_TAP" : 0x40,
            "DOUBLE_TAP" : 0x20,
            "ACTIVITY"   : 0x10,
            "INACTIVITY" : 0x08,
            "FREE_FALL"  : 0x04,
            "WATERMARK"  : 0x02,
            "OVERRUN"    : 0x01
        })

        self.data_format = Register(0x31, False, {
            "SELF_TEST"  : 0x80,
            "SPI"        : 0x40,
            "INT_INVERT" : 0x20,
            "FULL_RES"   : 0x08,
            "JUSITFY"    : 0x04,
            "RANGE"      : 0x03
        })

        self.fifo_status = Register(0x39, True, {
            "FIFO_TRIG"  : 0x80,  # 1 if trigger event has occured
            "ENTRIES"    : 0x3F   # Number of values stored in FIFO
        })

        #
        self.fifo_ctl = Register(0x38, False, {
            "MODE"       : 0xC0,
            # FIFO modes are defined by
            # 00: Bypass: FIFO is off and does not collect data
            # 01: FIFO: FIFO fills until full then stops.
            # 10: Stream: FIFO fills and loops over when full overwriting oldest data.
            # 11: Trigger: Holds data before and slightly after trigger event until full.
            "TRIGGER"    : 0x20,
            "SAMPLES"    : 0x1F
            # Using D4-D0, we can write a number between 1-31 that sets a watermark. This
            # watermark can act as an indicator that the FIFO has reached that number of
            # data values. When it has, the watermark interrupt is set, which can be 
            # found in the INT_SOURCE register
        })

        self._bind_registers()
        self._set_watermark()
        self._set_odr()
        self._set_range()

    def _bind_registers(self) -> None:
        """Bind all register objects to this device's bus."""
        for attr_name in dir(self):
            attr = getattr(self, attr_name)
            if isinstance(attr, Register):
                attr._bind(self.bus, self.address)


    def _set_watermark(self) -> None:
        self.fifo_ctl.write("SAMPLES", self.watermark)  # 28 by default


    def _set_odr(self) -> None:
        self.bandwidth_rate.write("RATE", self.odr)  # 100 Hz by default

    def _set_range(self) -> None:
        if self.g_range == Range.RANGE_FULL:
            self.data_format.write("FULL_RES", self.g_range)
        else:
            self.data_format.write("FULL_RES", 0b0) # Turn full range off
            self.data_format.write("RANGE", self.g_range)


    def set_watermark(self, watermark: int) -> None:
        self.watermark = watermark
        self._set_watermark()


    def set_odr(self, odr: OutputDataRate) -> None:
        self.odr = odr
        self._set_odr()


    def get_accel(self):
        """Read acceleration data from DATAX, DATAY, DATAZ registers.

        Returns:
            tuple: (x, y, z) acceleration values
        """
        REG_DATAX0 = 0x32
        SCALE_FACTOR = 0.0039 * 9.8  # g per LSB in FULL_RES mode (3.9 mg/LSB)

        # Read all 6 bytes at once
        data = self.bus.read_i2c_block_data(self.address, REG_DATAX0, 6)

        # Parse little-endian and convert to signed
        x = self._to_signed_16bit((data[1] << 8) | data[0])
        y = self._to_signed_16bit((data[3] << 8) | data[2])
        z = self._to_signed_16bit((data[5] << 8) | data[4])

        return (x * SCALE_FACTOR, y * SCALE_FACTOR, z * SCALE_FACTOR)

    def _to_signed_16bit(self, value):
        """Convert unsigned 16-bit value to signed."""
        if value & 0x8000:
            return value - 65536
        return value
