import serial
import time

from robot_lidar_alt import read_full_scan_from_serial


LIDAR_PORT="/dev/ttyUSB0"
LIDAR_BAUT = 230400
LIDAR_SERIAL_TIMEOUT = 0.2

ser = serial.Serial(LIDAR_PORT, LIDAR_BAUT, timeout=LIDAR_SERIAL_TIMEOUT)
try:
    result = read_full_scan_from_serial(
            ser,
            angle_offset=0.0,
            clockwise=False,
            max_revo_seconds=6.0,
        )
    time.
    with open("lidar_out.txt", "w+") as f:
        for k, v in result.items():
            print(f"{k}: {v}", file=f)
finally:
    try:
        ser.close()
    except Exception:
        pass
