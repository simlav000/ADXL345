#!/usr/bin/env python3

class Register:
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

class ADXL:
    def __init__(self, address: int, bus):
        self.address = address
        self.bus = bus

        # Define registers
        # WARNING: Registers 0x01 through 0x1C are reserved. Do not touch!
        self.bandwidth_rate = Register(0x2C, False, {
            "LOW_POWER"  : 0x10,
            "RATE"       : 0x0F # TODO: Figure out way to only pass legal values for rate.
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

    def _bind_registers(self):
        """Bind all register objects to this device's bus."""
        for attr_name in dir(self):
            attr = getattr(self, attr_name)
            if isinstance(attr, Register):
                attr._bind(self.bus, self.address)

    def get_accel(self):
        """Read acceleration data from DATAX, DATAY, DATAZ registers.

        Returns:
            tuple: (x, y, z) acceleration values
        """
        REG_DATAX0 = 0x32
        SCALE_FACTOR = 0.0039  # g per LSB in FULL_RES mode (3.9 mg/LSB)

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
