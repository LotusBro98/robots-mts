import numpy as np
from sock_utils import connect_robot, disconnect_robot, recv_tel, send_cmd
from drive_utils import rotate_rel, update_ref_angle, drive, rotate, stop, REF_ANGLE

RIGHT_TARGET = 0.25
SAFE_FRONT   = 0.30
OPEN_THRESH  = 0.55
STEP_DIST    = 0.3
SMOOTH_STOP  = 2
MAX_V        = 0.9
MIN_HOLE_MULT = 1.5

CORNER_GATE_FRONT  = 0.70   # разрешаем поворот, когда фронт стал «близко»
ALIGN_OFFSET       = 0.30   # ...или проехали столько после первого детекта «право открыто»
RIGHT_OPEN_HYST    = 3      # «право открыто» должно держаться N циклов подряд
FRONT_DROP_THRESH  = 0.12   # фронт за время подхода заметно упал (метрики — опционально)
# Защёлка "был проём" держится N циклов
LEFT_OPEN_LATCH_LEN  = 3
RIGHT_OPEN_LATCH_LEN = 3
# «Подъехать к углу»: до какого фронт-расстояния катимся перед левым поворотом
ADVANCE_FRONT_TARGET = 0.42   # остановимся, когда front <= это значение
ADVANCE_SMOOTH       = 0.40   # плавное торможение на последнем участке

def is_open(front, right, left):
    """Бинарные признаки доступности направлений (порог можно тонко подстроить)."""
    can_go_right = right > OPEN_THRESH
    can_go_front = front > OPEN_THRESH
    can_go_left  = left  > OPEN_THRESH
    return can_go_right, can_go_front, can_go_left


def advance_to_corner(front_target: float):
    """
    Едем ПРЯМО до тех пор, пока фронт не станет <= front_target.
    ВАЖНО: без прилипания к стенам (right_wall_dist=None), чтобы «дырa по диагонали» не сбивала движение.
    """
    print(f"[advance] approach corner until front <= {front_target:.3f}")
    drive(
        direction=0,                       # держим курс по REF_ANGLE
        max_dist=None,                     # не ограничиваем дистанцией, едем по фронту
        front_wall_dist=front_target,      # остановимся, когда front < target (≈ дойдём до угла)
        left_wall_dist=None,
        right_wall_dist=None,              # отключено: не реагируем на боковую «дыру» по диагонали
        reverse=False,
        max_speed=MAX_V,
        smooth_stop_dist=ADVANCE_SMOOTH
    )
    print("[advance] corner reached (front gate)")


def right_hand_maze(max_steps=10_000):
    """Правый обход с подъездом к углу как налево, так и направо (симметрия)."""
    update_ref_angle(0)
    global REF_ANGLE

    steps = 0
    left_latch  = 0
    right_latch = 0

    try:
        while steps < max_steps:
            # 1) сенсоры
            pos, th, vel, th_vel, gyro, ranges = recv_tel(REF_ANGLE)
            front = ranges[len(ranges)//2]
            right = ranges[0]
            left  = ranges[-1]

            can_right, can_front, can_left = is_open(front, right, left)
            print(f"[sense] front={front:.3f} right={right:.3f} left={left:.3f}  -> "
                  f"can_right={can_right} can_front={can_front} can_left={can_left}  "
                  f"left_latch={left_latch} right_latch={right_latch}")
            
            # 2) запуск защёлок
            if can_right:
                right_latch = RIGHT_OPEN_LATCH_LEN
                print(f"[latch] RIGHT opened -> start countdown: {right_latch}")
            elif right_latch > 0:
                right_latch -= 1

            if can_left:
                left_latch = LEFT_OPEN_LATCH_LEN
                print(f"[latch] LEFT opened while FRONT closed -> start countdown: {left_latch}")
            elif left_latch > 0:
                left_latch -= 1

            if can_right or right_latch > 0:
                if right_latch:
                    print("Помним что недавно был поворот направо - доезжаем немного и поворачиваем")
                else:
                    print("Заметили поворот направо - доезжаем немного и поворачиваем")
                update_ref_angle()
                drive(direction=0, max_dist=STEP_DIST, front_wall_dist=SAFE_FRONT, max_speed=MAX_V, smooth_stop_dist=SMOOTH_STOP)
                rotate_rel(-90)
                drive(
                    max_dist=STEP_DIST * 0.6,
                    front_wall_dist=SAFE_FRONT,
                    right_wall_dist=RIGHT_TARGET,
                    max_speed=MAX_V,
                    smooth_stop_dist=SMOOTH_STOP
                )
                right_latch = 0
                steps += 1

            elif can_front:
                print("Заметили что направо не можем, а вперёд можем - едем по правой стенке")
                drive(
                    max_dist=STEP_DIST,
                    front_wall_dist=SAFE_FRONT,
                    right_wall_dist=RIGHT_TARGET,  # можно держаться правой стены в узком коридоре
                    max_speed=MAX_V,
                    smooth_stop_dist=SMOOTH_STOP
                )

            elif can_left or left_latch > 0:
                if left_latch:
                    print("Помним что недавно был поворот налево - доезжаем немного и поворачиваем")
                else:
                    print("Заметили поворот налево - доезжаем немного и поворачиваем")
                update_ref_angle()
                drive(direction=0, max_dist=STEP_DIST, front_wall_dist=SAFE_FRONT, max_speed=MAX_V, smooth_stop_dist=SMOOTH_STOP)
                rotate_rel(+90)
                drive(
                    max_dist=STEP_DIST*0.6,
                    front_wall_dist=SAFE_FRONT,
                    right_wall_dist=RIGHT_TARGET if not right_latch else None,
                    max_speed=MAX_V,
                    smooth_stop_dist=SMOOTH_STOP
                )
                left_latch = 0

            else:
                print("Справа, спереди и сзади не можем ехать - разворот")
                rotate_rel(180)
                drive(
                    max_dist=STEP_DIST*0.6,
                    front_wall_dist=SAFE_FRONT,
                    right_wall_dist=RIGHT_TARGET,
                    max_speed=MAX_V,
                    smooth_stop_dist=SMOOTH_STOP
                )

            steps += 1

            # 6) эвристика выхода
            if front > 3.0 and right > 3.0:
                print("\n[maze] Likely exit detected — stopping.")
                break

    finally:
        stop()


if __name__ == "__main__":
    try:
        connect_robot()
        right_hand_maze()
    except RuntimeError as e:
        print(e)
    finally:
        disconnect_robot()
