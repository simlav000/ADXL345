import socket, time
from adxl345 import ADXL345, OutputDataRate
import smbus2


bus = smbus2.SMBus(1)
sensor = ADXL345(
    0x1D,
    bus,
    watermark=28,
    odr = OutputDataRate.ODR_100
)
s = socket.socket()
s.bind(("0.0.0.0", 9000))
s.listen(1)
conn, addr = s.accept()

while True:
    x, y, z = sensor.get_accel()
    conn.send(f"{x},{y},{z}\n".encode())
    time.sleep(1/100)  # match ODR
