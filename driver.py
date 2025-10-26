import time
import numpy as np

from navigator import Navigator
from robot_base import Robot


class Driver:
    MAX_ROT_SPEED = 1.0
    REGULATE_MAX_ANGLE = np.deg2rad(30)
    MIN_SPEED = 0.1
    MAX_SPEED = 1.0
    SMOOTH_STOP_DIST = 0.0

    def __init__(self, robot: Robot):
        self.robot = robot

    REF_ANGLE = 0

    def update_ref_angle(self, cur_value=0):
        sens = self.robot.recv_sensors()
        self.REF_ANGLE = sens.angle - np.deg2rad(cur_value)

    def drive(
        self,
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
        sens = self.robot.recv_sensors()
        start_pos = sens.pos

        if direction is not None:
            direction = np.deg2rad(direction)
        if max_speed is None:
            max_speed = self.MAX_SPEED
        if smooth_stop_dist is None:
            smooth_stop_dist = self.SMOOTH_STOP_DIST
        if reverse:
            max_speed = -max_speed

        assert left_wall_dist is None or right_wall_dist is None
        assert direction is None or (left_wall_dist is None and right_wall_dist is None)

        while True:
            sens = self.robot.recv_sensors()
            cur_front_wall_dist = min(rng for angle, rng in sens.lidar_ranges.items() if angle > 10 and angle < 10)
            cur_left_wall_dist = min(rng for angle, rng in sens.lidar_ranges.items() if angle > 30) * np.sin(np.deg2rad(45))
            cur_right_wall_dist = min(rng for angle, rng in sens.lidar_ranges.items() if angle < -30) * np.sin(np.deg2rad(45))
            vel_front = sens.vel[0]

            dist = np.linalg.norm(start_pos - sens.pos)
            if max_dist is not None and dist > max_dist:
                print("\n[drive] max dist reached")
                break

            if front_wall_dist is not None and cur_front_wall_dist < front_wall_dist:
                print("\n[drive] wall reached")
                break

            if (
                left_wall_dist is not None
                and stop_on_wall_hole is not None
                and cur_left_wall_dist > stop_on_wall_hole
            ):
                print("\n[drive] left wall hole reached")
                break

            if (
                right_wall_dist is not None
                and stop_on_wall_hole is not None
                and cur_right_wall_dist > stop_on_wall_hole
            ):
                print("\n[drive] right wall hole reached")
                break

            if smooth_stop_dist > 0:
                stop_dist = smooth_stop_dist
                if max_dist is not None:
                    stop_dist = min(max_dist - dist, stop_dist)
                if front_wall_dist is not None:
                    stop_dist = min(cur_front_wall_dist - front_wall_dist, stop_dist)
                t = np.clip(stop_dist / smooth_stop_dist, -1, 1)
                speed = np.sign(max_speed) * np.clip(
                    t * abs(max_speed), self.MIN_SPEED, abs(max_speed)
                )
                speed += 1 * (speed - vel_front)
            else:
                speed = max_speed

            if direction is not None:
                angle_diff = direction - sens.angle
                angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
                t = angle_diff / self.REGULATE_MAX_ANGLE
            elif (
                left_wall_dist is not None and cur_left_wall_dist < cur_front_wall_dist
            ):
                t = (cur_left_wall_dist - left_wall_dist) / left_wall_dist * 2
            elif (
                right_wall_dist is not None
                and cur_right_wall_dist < cur_front_wall_dist
            ):
                t = -(cur_right_wall_dist - right_wall_dist) / right_wall_dist * 2
            else:
                t = 0
            rot = np.clip(
                t * self.MAX_ROT_SPEED, -self.MAX_ROT_SPEED, self.MAX_ROT_SPEED
            )

            msg = f"\r[drive] "
            msg += f"dist: {dist:6.3f} speed: {speed:6.3f} vel: {vel_front:6.3f} "
            if direction is not None:
                msg += f"th: {sens.angle:6.3f} rot_speed: {rot:6.3f} "
            if front_wall_dist is not None:
                msg += f"front_wall: {cur_front_wall_dist:6.3f} "
            if left_wall_dist is not None:
                msg += f"left_wall: {cur_left_wall_dist:6.3f} "
            if right_wall_dist is not None:
                msg += f"right_wall: {cur_right_wall_dist:6.3f} "
            print(msg, end="", flush=True)
            self.robot.send_drive(speed, rot)

        print("[drive] stop")
        self.robot.send_drive(0, 0)

    def rotate(
        self,
        direction,
        relative=False,
    ):
        """
        direction - в градусах относительно ref_angle(). влево +, вправо -
        relative - относительно текущего положения, или относительно ref_angle()
        """
        ANGLE_THRESHOLD = np.deg2rad(1)

        sens = self.robot.recv_sensors()
        direction = np.deg2rad(direction)
        if relative:
            direction = sens.angle + direction

        while True:
            sens = self.robot.recv_sensors()

            angle_diff = direction - sens.angle
            angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
            if abs(angle_diff) < ANGLE_THRESHOLD:
                print("\n[rotate] target angle reached")
                break

            t = np.clip(angle_diff / self.REGULATE_MAX_ANGLE, -1, 1)
            rot = np.clip(
                t * self.MAX_ROT_SPEED, -self.MAX_ROT_SPEED, self.MAX_ROT_SPEED
            )

            print(f"\r[rotate] th: {sens.angle:6.3f} rot_speed: {rot:6.3f}", end="", flush=True)
            self.robot.send_drive(0, rot)

        print("[rotate] stop")
        self.robot.send_drive(0, 0)

    def stop(self):
        th_threshold = 0.01
        vel_threshold = 0.01
        """Sends stop and wait it stopped."""

        while True:
            sens = self.robot.recv_sensors()

            if abs(sens.angle_vel) < th_threshold and np.linalg.norm(sens.vel) < vel_threshold:
                print("\n[stop] position stabilized")
                break

            print(
                f"\r[stop] vel: {np.linalg.norm(sens.vel):6.3f} vel_th: {sens.angle_vel:6.3f}",
                end="",
                flush=True,
            )
            self.robot.send_drive(0, 0)

        print("[stop] stopped")
        self.robot.send_drive(0, 0)

    def freeze(self):
        vel_threshold = 0.01
        k = 1
        eps = 0.05
        """Return back to position where it was called, decrease inertia"""

        sens = self.robot.recv_sensors()
        start_pos = sens.pos

        while True:
            sens = self.robot.recv_sensors()
            delta = np.dot(sens.pos - start_pos, np.array([np.cos(sens.angle), np.sin(sens.angle)]))

            if abs(delta) < eps and np.linalg.norm(sens.vel) < vel_threshold:
                print("\n[freeze] returned to previous position")
                break

            speed = np.clip(-delta * k, -self.MAX_SPEED, self.MAX_SPEED)

            print(
                f"\r[freeze] delta: {delta:6.3f} vel: {np.linalg.norm(sens.vel):6.3f}",
                end="",
                flush=True,
            )
            self.robot.send_drive(speed, 0)

        print("[freeze] stopped")
        self.robot.send_drive(0, 0)

    def drive_maze(
        self,
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
        sens = self.robot.recv_sensors()
        start_pos = sens.pos

        assert left_wall_dist is None or right_wall_dist is None

        while True:
            sens = self.robot.recv_sensors()
            right_pt = -15
            left_pt = 15
            cur_front_wall_dist = min(rng for angle, rng in sens.lidar_ranges.items() if angle < left_pt and angle > right_pt)
            cur_left_wall_dist = min(rng for angle, rng in sens.lidar_ranges.items() if angle > left_pt) * np.sin(np.deg2rad(45))
            cur_right_wall_dist = min(rng for angle, rng in sens.lidar_ranges.items() if angle < right_pt) * np.sin(np.deg2rad(45))
            vel_front = sens.vel[0]

            dist = np.linalg.norm(start_pos - sens.pos)

            # if left_wall_dist is not None and cur_left_wall_dist > left_wall_dist + side_wall_smooth_stop_dist:
            #     print("\n[drive] lost left wall")
            #     break
            # if right_wall_dist is not None and cur_right_wall_dist > right_wall_dist + side_wall_smooth_stop_dist:
            #     print("\n[drive] lost right wall")
            #     break

            if left_wall_dist is not None:
                t = (cur_left_wall_dist - left_wall_dist) / side_wall_smooth_stop_dist
            elif right_wall_dist is not None:
                t = (
                    -(cur_right_wall_dist - right_wall_dist)
                    / side_wall_smooth_stop_dist
                )
            else:
                t = 0
            rot = np.clip(t * self.MAX_ROT_SPEED, -self.MAX_ROT_SPEED, self.MAX_ROT_SPEED)

            stop_dist = cur_front_wall_dist - front_wall_dist
            t = np.clip(stop_dist / front_wall_smooth_stop_dist, -1, 1)
            speed = (
                np.sign(max_speed)
                * np.sign(t)
                * np.clip(abs(t) * abs(max_speed), self.MIN_SPEED, abs(max_speed))
            )
            speed += 1 * (speed - vel_front)

            speed *= 1 - 0.8 * abs(rot / self.MAX_ROT_SPEED)
            rot = np.clip(
                rot * abs(max_speed) / abs(speed), -self.MAX_ROT_SPEED, self.MAX_ROT_SPEED
            )

            msg = f"\r[drive] "
            msg += f"dist: {dist:6.3f} speed: {speed:6.3f} vel: {vel_front:6.3f} "
            if left_wall_dist is not None or right_wall_dist is not None:
                msg += f"th: {sens.angle:6.3f} rot_speed: {rot:6.3f} "
            if front_wall_dist is not None:
                msg += f"front_wall: {cur_front_wall_dist:6.3f} "
            if left_wall_dist is not None:
                msg += f"left_wall: {cur_left_wall_dist:6.3f} "
            if right_wall_dist is not None:
                msg += f"right_wall: {cur_right_wall_dist:6.3f} "
            print(msg, end="", flush=True)
            # speed = 0.05
            # rot = 1
            self.robot.send_drive(speed, rot)
            time.sleep(0.1)

            self.robot.navigator.display()

        print("[drive] stop")
        self.robot.send_drive(0, 0)
