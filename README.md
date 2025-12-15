# ADXL345 Python Drivers
These drivers were written to solve precise timing issues arising from
the CircuitPython adafruit drivers in order to perform frequency-based vibration
analysis.

## A bit of context
The ADXL345's FIFO is a memory space capable of holding 32 accelerometer
triplets (x, y, z) sorted in a First-In-First-Out manner which are equally
spaced in time (up to the chip's specifications), but these data are not
directly accessible. Instead, a certain set of registers (`0x32` through
`0x37`) hold the oldest value previously in the FIFO, meaning the chip can
ostensibly store exactly 32 + 1 accelerometer triplets. These special registers
are arranged as:
 | Meaning | Register address | Register name |
 | ------- | ---------------- | ------------- |
 | X_LSB   | `0x32`           | `DATAX0`      |
 | X_MSB   | `0x33`           | `DATAX1`      |
 | Y_LSB   | `0x34`           | `DATAY0`      |
 | Y_MSB   | `0x35`           | `DATAY1`      |
 | Z_LSB   | `0x36`           | `DATAZ0`      |
 | Z_MSB   | `0x37`           | `DATAZ1`      |
 
Where LSB means least significant byte and MSB means most significant byte.
Thus, each axis comprises a 2 byte floating point number. The
`accelerometer.acceleration` method from the CircuitPython library reads the
contents of registers `0x32 - 0x37` and upon a successful read, the oldest value
in the FIFO is taken out, and placed in these registers.

### The problem at hand
Basically, the out-of-the-box drivers give you access to the
`accelerometer.acceleration` method, but does not expose the chip's FIFO status
and control registers in an easy-to-read or write manner, which is instrumental
information to ensure that acceleration samples are equally spaced in time. If
these checks are not done, there are two potential avenues for unevenly sampled
data

 1. **Reading too fast**:
    If the program reads faster than the chip's output data rate, the FIFO will
    be empty when the next read is performed, and so there will be no data to 
    replace the contents of registers `0x32 - 0x37` with. This means that data
    will be duplicated.
 2. **Reading too slow**:
    If the program reads too slowly, the FIFO will fill up beyond its capacity,
    and it will discard the oldest data, meaning there is no longer a guarantee
    that two values from `accel.acceleration` are equally spaced in time.

The first problem can be solved trivially with access to the `0x39 -
FIFO_STATUS` register's "ENTRIES" field, which simply holds the number of 
values currently stored in the FIFO. With this, we can just `time.sleep(1/ODR)` 
where ODR is the output data rate to give the FIFO has enough time to fill up
with data when we find that it is empty.

The second problem also addressed by monitoring the number of values in the 
FIFO and alerting the user if data has been discarded. To monitor this, a
FIFO status bar is printed which looks like:
```[█████████████████████████████████░░|░░░░] 27/32 Samples: 1337```
which simply helps visualize how full the FIFO is. The vertical bar towards
the end designates the watermark, and once the FIFO is completely filled,
a warning message is printed and the event is counted. At the end of the 
measurement run, the total number of overflow events is printed.

This program works without overflow for most ODRs 
but once we reach ODRs beyond ~800 Hz, we begin to be bottlenecked by
i2c and require SPI to read faster. Since the vibrations I am trying to read
are of the order of ~60 Hz, I do not care about this.

### Usage
If on raspberry pi, run `i2cdetect -y 1` to  obtain device address.

Then, in your script, write:
```
from adxl345 import ADXL, OutputDataRate
import smbus2

# Optional parameters, defaults are already
WATERMARK = 28 
ODR = OutputDataRate.ODR_100 

adxl = ADXL(DEVICE_ADDRESS, smbus2.SMBus(1), WATERMARK, ODR)

x, y, z = adxl.get_accel()
```
use the above for simple operation, and see `measure.py` for precise
timing usage.
