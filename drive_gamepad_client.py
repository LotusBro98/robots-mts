#!/usr/bin/env python3
import socket
import struct
import time
import pygame

# ==== НАСТРОЙКИ СЕТИ ====
ROBOT_IP = "192.168.1.11"   # <-- сюда IP робота
ROBOT_PORT = 9999

# ==== НАСТРОЙКИ СКОРОСТЕЙ ====
MAX_LINEAR_SPEED = 1.0   # м/с, подстрой под своего робота
MAX_ANGULAR_SPEED = 1.0  # рад/с

# ==== ОСИ ГЕЙМПАДА (МОЖЕШЬ ПОДШУММИТЬ ПОД СВОЙ) ====
AXIS_LINEAR = 1   # обычно левый стик: вертикальная ось
AXIS_ANGULAR = 0  # обычно левый стик: горизонтальная ось

DEADZONE = 0.1    # мёртвая зона стика

def apply_deadzone(value, deadzone=0.1):
    if abs(value) < deadzone:
        return 0.0
    return value

def main():
    # --- сеть ---
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    packet_format = "ff"

    # --- pygame / геймпад ---
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("Геймпад не найден. Подключи и перезапусти скрипт.")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Используем геймпад: {joystick.get_name()}")

    clock = pygame.time.Clock()

    try:
        while True:
            # Обработка событий (иначе оси не обновляются)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt
                if event.type == pygame.JOYBUTTONDOWN:
                    # Кнопка B/круглый — выход (пример)
                    # Посмотри индексы кнопок через print, если нужно.
                    pass

            # Чтение осей
            axis_lin_raw = joystick.get_axis(AXIS_LINEAR)   # -1 вперед / +1 назад обычно
            axis_ang_raw = joystick.get_axis(AXIS_ANGULAR)  # -1 влево / +1 вправо

            axis_lin = apply_deadzone(axis_lin_raw, DEADZONE)
            axis_ang = apply_deadzone(axis_ang_raw, DEADZONE)

            # Обычно ось линейного инвертирована: -1 = вперед
            linear_speed = -axis_lin**3 * MAX_LINEAR_SPEED
            angular_speed = -axis_ang**3 * MAX_ANGULAR_SPEED

            # Пакуем и отправляем
            data = struct.pack(packet_format, linear_speed, angular_speed)
            sock.sendto(data, (ROBOT_IP, ROBOT_PORT))

            # Ограничиваем частоту отправки, например 30 Гц
            clock.tick(30)

    except KeyboardInterrupt:
        print("\nОстанов по Ctrl+C, отправляем стоп...")
        # отправим нули пару раз на всякий
        for _ in range(5):
            data = struct.pack(packet_format, 0.0, 0.0)
            sock.sendto(data, (ROBOT_IP, ROBOT_PORT))
            time.sleep(0.02)

    finally:
        pygame.joystick.quit()
        pygame.quit()
        sock.close()
        print("Клиент завершён.")

if __name__ == "__main__":
    main()