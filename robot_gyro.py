#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LSM6DS3 (SPI, Raspberry Pi) — измеряем только yaw:
- Проверка наличия SPI
- WHO_AM_I == 0x69
- Настройка акселя/гиро
- Калибровка нуля gz (усреднение)
- Вывод gz (°/s) и интегрированного угла вокруг оси Z (°)
"""

import glob
import math
import time
import spidev

# --- SPI протокол LSM6DS3 ---
RD = 0x80  # bit7=1 -> read
AI = 0x40  # bit6=1 -> auto-increment

# --- Регистры LSM6DS3 ---
WHO_AM_I  = 0x0F
CTRL1_XL  = 0x10
CTRL2_G   = 0x11
CTRL3_C   = 0x12
OUTX_L_G  = 0x22   # GX_L..GZ_H (6 байт)
OUTX_L_XL = 0x28   # AX_L..AZ_H (6 байт)

EXPECTED_WHOAMI = 0x69

def twos_compl(lo, hi):
    v = lo | (hi << 8)
    return v - 0x10000 if v & 0x8000 else v

class LSM6DS3SPI:
    def __init__(self, bus=0, dev=0, max_hz=5_000_000):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, dev)
        self.spi.max_speed_hz = max_hz
        self.spi.mode = 0b00
        self.spi.cshigh = False

        who = self.read_reg(WHO_AM_I)
        if who != EXPECTED_WHOAMI:
            self.spi.close()
            raise RuntimeError(f"WHO_AM_I=0x{who:02X}, ожидается 0x{EXPECTED_WHOAMI:02X} — датчик не LSM6DS3?")

        # IF_INC=1 (auto-increment), BDU=1 (фиксируем данные до чтения обоих байтов)
        self.write_reg(CTRL3_C, 0b01000100)

        # Настройка частоты/диапазонов:
        # Accel: ODR=104 Гц (0100), FS=±4g (10), BW=50 Гц (00)
        self.write_reg(CTRL1_XL, 0b01000010)
        # Gyro:  ODR=104 Гц (0100), FS=±1000 dps (10)
        self.write_reg(CTRL2_G,  0b01001000)

        # Чувствительности по даташиту:
        self.g_sens_dps_per_lsb = 0.035      # ±1000 dps → 35 mdps/LSB
        # (аксель нам тут не нужен, но оставлю на будущее)
        # 0.122 mg/LSB → м/с²:
        self.a_sens_ms2_per_lsb = 0.122e-3 * 9.80665

    def close(self):
        self.spi.close()

    def read_reg(self, reg):
        rx = self.spi.xfer2([reg | RD, 0x00])
        return rx[1]

    def write_reg(self, reg, val):
        self.spi.xfer2([reg & 0x7F, val & 0xFF])

    def read_gyro_dps(self):
        b = self.spi.xfer2([OUTX_L_G | RD | AI] + [0x00]*6)[1:]
        gx = twos_compl(b[0], b[1]) * self.g_sens_dps_per_lsb
        gy = twos_compl(b[2], b[3]) * self.g_sens_dps_per_lsb
        gz = twos_compl(b[4], b[5]) * self.g_sens_dps_per_lsb
        return gx, gy, gz

def ensure_spi():
    if not glob.glob("/dev/spidev*"):
        raise RuntimeError("SPI не включён. Выполни: sudo raspi-config → Interface Options → SPI → Enable")

def calibrate_gz(imu, seconds=2.0):
    """
    Быстро оцениваем нулевой сдвиг гироскопа по Z:
    усредняем gz, пока робот неподвижен.
    """
    print(f"Калибрую gz {seconds:.1f} c — не трогай робота…")
    t0 = time.monotonic()
    s = n = 0
    while time.monotonic() - t0 < seconds:
        _, _, gz = imu.read_gyro_dps()
        s += gz
        n += 1
        time.sleep(0.005)
    bias = s / max(n, 1)
    print(f"Смещение gz ≈ {bias:.3f} °/с")
    return bias

def wrap_angle_deg(a):
    """Нормализация угла в диапазон (-180, 180] для удобства чтения."""
    a = (a + 180.0) % 360.0 - 180.0
    return a

def init_imu(bus: int = 0, dev: int = 0, max_hz: int = 5_000_000) -> LSM6DS3SPI:
    """
    Инициализация гироскопа LSM6DS3 по SPI.
    Проверяет наличие SPI и возвращает готовый объект IMU.
    """
    ensure_spi()
    imu = LSM6DS3SPI(bus=bus, dev=dev, max_hz=max_hz)
    print("IMU инициализирован: ODR=104 Гц, FS_g=±1000 dps.")
    return imu

def read_yaw_rate_and_angle(imu: LSM6DS3SPI, gz_bias: float, angle_z: float, t_prev: float):
    """
    Считывает текущую скорость поворота вокруг оси Z и интегрирует угол.

    :param imu: объект LSM6DS3SPI
    :param gz_bias: текущее смещение гироскопа (°/с)
    :param angle_z: предыдущий интегрированный угол (°)
    :param t_prev: предыдущий момент времени
    :return: (gz_corr, angle_z, t_now)
    """
    gx, gy, gz = imu.read_gyro_dps()
    t_now = time.monotonic()
    dt = t_now - t_prev
    gz_corr = gz - gz_bias
    angle_z += gz_corr * dt
    return gz_corr, angle_z, t_now

def recalibrate_imu(imu: LSM6DS3SPI, seconds: float = 1.0) -> float:
    """
    Быстрая перекалибровка гироскопа: усредняет смещение gz в покое.
    Возвращает новое значение bias.
    """
    print(f"Перекалибровка IMU ({seconds:.1f} c)... Робот должен стоять неподвижно.")
    t0 = time.monotonic()
    s = n = 0
    while time.monotonic() - t0 < seconds:
        _, _, gz = imu.read_gyro_dps()
        s += gz
        n += 1
        time.sleep(0.005)
    new_bias = s / max(n, 1)
    print(f"Новый bias gz ≈ {new_bias:.3f} °/с")
    return new_bias


def main():
    ensure_spi()

    # Обычно /dev/spidev0.0
    bus, dev = 0, 0
    imu = LSM6DS3SPI(bus, dev, max_hz=5_000_000)
    print("LSM6DS3 готов: ODR=104 Гц, FS_g=±1000 dps.")

    # Быстрая калибровка нуля yaw-скорости
    gz_bias = calibrate_gz(imu, seconds=2.0)

    angle_z = 0.0   # интегрированный курс (°)
    t_prev = time.monotonic()

    try:
        while True:
            gx, gy, gz = imu.read_gyro_dps()          # °/с
            now = time.monotonic()
            dt = now - t_prev
            t_prev = now

            gz_corr = gz - gz_bias                    # вычитаем смещение
            angle_z += gz_corr * dt                   # интегрируем

            # Красиво печатаем:
            print(f"Yaw speed: {gz_corr:+7.2f} °/с | Angle Z: {angle_z:+7.2f} ° | AngleZ(norm): {wrap_angle_deg(angle_z):+7.2f} °")

            time.sleep(0.01)  # ≈100 Гц печать
    finally:
        imu.close()

if __name__ == "__main__":
    main()
