import numpy as np
from drive_utils import REF_ANGLE, drive, freeze, rotate, update_ref_angle
from sock_utils import connect_robot, disconnect_robot, recv_tel, send_cmd

MAX_ROT_SPEED = 1.0
MIN_SPEED = 0.001


def drive_maze(
        max_speed,
        front_wall_dist,
        front_wall_smooth_stop_dist,
        side_wall_smooth_stop_dist,
        left_wall_dist=None,
        right_wall_dist=None,
    ):
    """
    direction - в градусах относительно ref_angle(). влево +, вправо -
    max_dist - на какое расстояние проехать
    front_wall_dist - остановиться, если стенка ближе чем это расстояние
    smooth_stop_dist - расстояние до конца, на котором начать плавно тормозить
    reverse - вперёд или назад
    """
    pos, th, vel, th_vel, gyro, ranges = recv_tel(REF_ANGLE)
    start_pos = pos

    assert left_wall_dist is None or right_wall_dist is None

    while True:
        pos, th, vel, th_vel, gyro, ranges = recv_tel(REF_ANGLE)
        left_pt = int(len(ranges) * 0.3)
        right_pt = int(len(ranges) * 0.3)
        left_pt_front = int(len(ranges) * 0.45)
        right_pt_front = int(len(ranges) * 0.45)
        cur_front_wall_dist = min(ranges[right_pt_front:-left_pt_front])
        cur_left_wall_dist = min(ranges[-left_pt:]) * np.sin(np.deg2rad(45))
        cur_right_wall_dist = min(ranges[:right_pt]) * np.sin(np.deg2rad(45))
        vel_front = vel[0]

        dist = np.linalg.norm(start_pos - pos)
        front_wall_smooth_stop_dist_v = front_wall_smooth_stop_dist * np.linalg.norm(vel).clip(0.1, None)

        # if left_wall_dist is not None and cur_left_wall_dist > left_wall_dist + side_wall_smooth_stop_dist:
        #     print("\n[drive] lost left wall")
        #     break
        # if right_wall_dist is not None and cur_right_wall_dist > right_wall_dist + side_wall_smooth_stop_dist:
        #     print("\n[drive] lost right wall")
        #     break


        if left_wall_dist is not None:
            t = (cur_left_wall_dist - left_wall_dist) / side_wall_smooth_stop_dist
        elif right_wall_dist is not None:
            t = -(cur_right_wall_dist - right_wall_dist) / side_wall_smooth_stop_dist
        else:
            t = 0
        rot = np.clip(t * MAX_ROT_SPEED, -MAX_ROT_SPEED, MAX_ROT_SPEED)

        stop_dist = cur_front_wall_dist - front_wall_dist
        t = np.clip(stop_dist / front_wall_smooth_stop_dist_v, -1, 1)
        speed = np.sign(max_speed) * np.sign(t) * np.clip(abs(t) * abs(max_speed), MIN_SPEED, abs(max_speed))
        speed += 1 * (speed - vel_front)

        speed *= 1 - 0.8 * abs(rot / MAX_ROT_SPEED)
        rot = np.clip(rot * abs(max_speed) / abs(speed), -MAX_ROT_SPEED, MAX_ROT_SPEED)

        msg = f"\r[drive] "
        msg += f"dist: {dist:6.3f} speed: {speed:6.3f} vel: {vel_front:6.3f} "
        if left_wall_dist is not None or right_wall_dist is not None:
            msg += f"th: {th:6.3f} rot_speed: {rot:6.3f} "
        if front_wall_dist is not None:
            msg += f"front_wall: {cur_front_wall_dist:6.3f} "
        if left_wall_dist is not None:
            msg += f"left_wall: {cur_left_wall_dist:6.3f} "
        if right_wall_dist is not None:
            msg += f"right_wall: {cur_right_wall_dist:6.3f} "
        print(msg, end="", flush=True)
        send_cmd(speed, rot)

    print("[drive] stop")
    send_cmd(0, 0)


try:
    connect_robot()
    update_ref_angle()

    while True:
        drive_maze(
            max_speed=0.4,
            front_wall_dist=0.25,
            right_wall_dist=0.25, 
            front_wall_smooth_stop_dist=4,
            side_wall_smooth_stop_dist=0.2
        )
        freeze()
except Exception as e:
    print(e)
finally:
    disconnect_robot()
