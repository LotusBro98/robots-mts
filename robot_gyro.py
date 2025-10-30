import time
import board
import busio
from adafruit_bno055 import BNO055_I2C

i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO055_I2C(i2c, address=0x28)  # если не видит — попробуйте 0x29

# режим по умолчанию NDOF уже даёт fusion-данные
# полезно проверять калибровку (sys, gyro, accel, mag: 0..3)
while True:
    euler = bno.euler        # (heading, roll, pitch) в градусах или None, если нет
    quat  = bno.quaternion   # (w, x, y, z)
    gyro  = bno.gyro         # (x, y, z) в rad/s
    accel = bno.acceleration # (x, y, z) в m/s^2
    cal   = bno.calibration_status
    print("Euler:", euler, "Gyro:", gyro, "Cal:", cal)
    time.sleep(0.02)
