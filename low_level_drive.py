"""
Это адаптер от chassis.py до привычной функции управления моторами send_cmd(),
которая используется в drive_utils.py
"""
from chassis import RobotChassis


def connect_robot():
    global robot
    robot = RobotChassis("/dev/ttyACM1")
    robot.connect()

def send_cmd(v: float, w: float):
    """
    v: float — линейная скорость вперёд (0..1)
    w: float — угловая скорость (поворот) влево/вправо (-1..1)
              >0 — влево, <0 — вправо

    Моторы управляются в диапазоне -18000..18000 (единицы 0.1 rpm).
    """
    MAX_RPM = 1800
    SCALE = MAX_RPM * 10  # перевод в 0.1rpm

    v = max(-1.0, min(1.0, v))
    w = max(-1.0, min(1.0, w))

    # (v, w) → (left, right)
    # поворот влево: левый мотор медленнее, правый быстрее
    left_speed  = (v - w) * SCALE
    right_speed = (v + w) * SCALE

    left_speed  = int(max(-MAX_RPM * 10, min(MAX_RPM * 10, left_speed)))
    right_speed = int(max(-MAX_RPM * 10, min(MAX_RPM * 10, right_speed)))
    robot.send_command(T=131, L=left_speed, R=right_speed)

def disconnect_robot():
    robot.disconnect()
