from smbus2 import i2c_msg, SMBus
from adxl345 import OutputDataRate, ADXL

DATAX0 = 0x32
BYTES_PER_SAMPLE = 6

def read_fifo(adxl):
    # Number of samples currently in FIFO (0–32)
    n = adxl.fifo_status.read("ENTRIES")
    if n == 0:
        return []

    nbytes = BYTES_PER_SAMPLE * n
    bus  = adxl.bus
    addr = adxl.addr

    write = i2c_msg.write(addr, [DATAX0])
    read  = i2c_msg.read(addr, nbytes)
    bus.i2c_rdwr(write, read)

    data = bytes(read)

    # Split into 6-byte samples
    return [
        data[i:i + 6]
        for i in range(0, len(data), 6)
    ]

adxl = ADXL(0X1D, SMBus(1), 28, OutputDataRate.ODR_3200)
adxl.data_format.write("FULL_RES", 1)
adxl.fifo_ctl.write("MODE", 0b10)          # STREAM mode
adxl.power_control.write("MEASURE", 1)     # START sampling

import time
time.sleep(0.01)   # 10 ms → FIFO can fill
print(read_fifo(adxl))


