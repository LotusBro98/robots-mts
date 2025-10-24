from typing import Union
import numpy as np

from sock_utils import recv_tel, send_cmd

REF_ANGLE = 0

def update_ref_angle(cur_value=0):
    pos, th, vel, th_vel, gyro, ranges = recv_tel()
    global REF_ANGLE
    REF_ANGLE = th - np.deg2rad(cur_value)


def rotate_rel(delta_deg: float):
    """Повернуть на относительный угол delta_deg (влево + / вправо -) безопасно.
    Делает абсолютный таргет в градусах относительно REF_ANGLE.
    """
    pos, th, vel, th_vel, gyro, ranges = recv_tel(REF_ANGLE)      # th уже «с учётом» REF_ANGLE
    cur_deg = np.rad2deg(th)
    rotate(direction=cur_deg + delta_deg, relative=False)          # абсолютная цель в градусах


MAX_ROT_SPEED = 1.0
REGULATE_MAX_ANGLE = np.deg2rad(30)
MIN_SPEED = 0.1
MAX_SPEED = 1.0
SMOOTH_STOP_DIST=0.0


def drive(
        direction=None, 
        max_dist=None, 
        front_wall_dist=None,
        left_wall_dist=None,
        right_wall_dist=None,
        reverse=False,
        max_speed=None,
        smooth_stop_dist=None,
        stop_on_wall_hole=None,
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

    if direction is not None:
        direction = np.deg2rad(direction)
    if max_speed is None:
        max_speed = MAX_SPEED
    if smooth_stop_dist is None:
        smooth_stop_dist = SMOOTH_STOP_DIST
    if reverse:
        max_speed = -max_speed

    assert left_wall_dist is None or right_wall_dist is None
    assert direction is None or (left_wall_dist is None and right_wall_dist is None)


    while True:
        pos, th, vel, th_vel, gyro, ranges = recv_tel(REF_ANGLE)
        cur_front_wall_dist = ranges[len(ranges) // 2]
        cur_left_wall_dist = ranges[-1] * np.sin(np.deg2rad(45))
        cur_right_wall_dist = ranges[0] * np.sin(np.deg2rad(45))
        vel_front = vel[0]

        dist = np.linalg.norm(start_pos - pos)
        if max_dist is not None and dist > max_dist:
            print("\n[drive] max dist reached")
            break

        if front_wall_dist is not None and cur_front_wall_dist < front_wall_dist:
            print("\n[drive] wall reached")
            break

        if left_wall_dist is not None and stop_on_wall_hole is not None and cur_left_wall_dist > stop_on_wall_hole:
            print("\n[drive] left wall hole reached")
            break

        if right_wall_dist is not None and stop_on_wall_hole is not None and cur_right_wall_dist > stop_on_wall_hole:
            print("\n[drive] right wall hole reached")
            break

        if smooth_stop_dist > 0:
            stop_dist = smooth_stop_dist
            if max_dist is not None:
                stop_dist = min(max_dist - dist, stop_dist)
            if front_wall_dist is not None:
                stop_dist = min(cur_front_wall_dist - front_wall_dist, stop_dist)
            t = np.clip(stop_dist / smooth_stop_dist, -1, 1)
            speed = np.sign(max_speed) * np.clip(t * abs(max_speed), MIN_SPEED, abs(max_speed))
            speed += 1 * (speed - vel_front)
        else:
            speed = max_speed

        if direction is not None:
            angle_diff = np.arctan2(np.sin(direction - th), np.cos(direction - th))
            t = angle_diff / REGULATE_MAX_ANGLE
        elif left_wall_dist is not None and cur_left_wall_dist < cur_front_wall_dist:
            t = (cur_left_wall_dist - left_wall_dist) / left_wall_dist * 2
        elif right_wall_dist is not None and cur_right_wall_dist < cur_front_wall_dist:
            t = -(cur_right_wall_dist - right_wall_dist) / right_wall_dist * 2
        else:
            t = 0
        rot = np.clip(t * MAX_ROT_SPEED, -MAX_ROT_SPEED, MAX_ROT_SPEED)

        msg = f"\r[drive] "
        msg += f"dist: {dist:6.3f} speed: {speed:6.3f} vel: {vel_front:6.3f} "
        if direction is not None:
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


def rotate(
        direction,
        relative=False,
    ):
    """
    direction - в градусах относительно ref_angle(). влево +, вправо -
    relative - относительно текущего положения, или относительно ref_angle()
    """
    ANGLE_THRESHOLD = np.deg2rad(1)

    pos, th, vel, th_vel, gyro, ranges = recv_tel(REF_ANGLE)
    direction = np.deg2rad(direction)
    if relative:
        direction = th + direction

    while True:
        pos, th, vel, th_vel, gyro, ranges = recv_tel(REF_ANGLE)

        angle_diff = np.arctan2(np.sin(direction - th), np.cos(direction - th))
        if abs(angle_diff) < ANGLE_THRESHOLD:
            print("\n[rotate] target angle reached")
            break

        t = np.clip(angle_diff / REGULATE_MAX_ANGLE, -1, 1)
        rot = np.clip(t * MAX_ROT_SPEED, -MAX_ROT_SPEED, MAX_ROT_SPEED)

        print(f"\r[rotate] th: {th:6.3f} rot_speed: {rot:6.3f}", end="", flush=True)
        send_cmd(0, rot)

    print("[rotate] stop")
    send_cmd(0, 0)


def stop():
    th_threshold=0.01
    vel_threshold=0.01
    """Sends stop and wait it stopped."""

    while True:
        pos, th, vel, th_vel, gyro, ranges = recv_tel(REF_ANGLE)

        if abs(th_vel) < th_threshold and np.linalg.norm(vel) < vel_threshold:
            print("\n[stop] position stabilized")
            break

        print(f"\r[stop] vel: {np.linalg.norm(vel):6.3f} vel_th: {th_vel:6.3f}", end="", flush=True)
        send_cmd(0, 0)

    print("[stop] stopped")
    send_cmd(0, 0)


def freeze():
    vel_threshold=0.01
    k = 1
    eps = 0.05
    """Return back to position where it was called, decrease inertia"""

    pos, th, vel, th_vel, gyro, ranges = recv_tel(0)
    start_pos = pos

    while True:
        pos, th, vel, th_vel, gyro, ranges = recv_tel(0)
        delta = np.dot(pos - start_pos, np.array([np.cos(th), np.sin(th)]))

        if abs(delta) < eps and np.linalg.norm(vel) < vel_threshold:
            print("\n[freeze] returned to previous position")
            break

        speed = np.clip(-delta * k, -MAX_SPEED, MAX_SPEED)

        print(f"\r[freeze] delta: {delta:6.3f} vel: {np.linalg.norm(vel):6.3f}", end="", flush=True)
        send_cmd(speed, 0)

    print("[freeze] stopped")
    send_cmd(0, 0)
