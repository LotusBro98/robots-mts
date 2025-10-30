import time
from matplotlib import pyplot as plt
import numpy as np

from navigator_utils import direction_vec, normalize, project_scalar, round_angle, transform_points
from robot_base import Robot


class Driver:
    MAX_ROT_SPEED = 1.0
    REGULATE_MAX_ANGLE = np.deg2rad(30)
    MIN_SPEED = 0.1
    MAX_SPEED = 1.0
    SMOOTH_STOP_DIST = 0.0
    ANGLE_THRESHOLD = np.deg2rad(5)
    MAZE_TURN_SPEED = 0.15
    MAZE_TURN_RADIUS = 0.25
    CONTROLLER_PERIOD = 0.05

    def __init__(self, robot: Robot):
        self.robot = robot

    REF_ANGLE = 0

    def update_ref_angle(self, cur_value=0):
        sens = self.robot.recv_sensors()
        self.REF_ANGLE = sens.angle - np.deg2rad(cur_value)

    def estimate_brake_distance(self, current_vel, target_vel):
        current_vel = current_vel * self.robot.MAX_SPEED_MpS
        target_vel = target_vel * self.robot.MAX_SPEED_MpS
        dist = abs(current_vel**2 - target_vel**2) / (2 * self.robot.MAX_ACCELERATION)
        return dist
    
    def estimate_angular_brake_distance(self, current_vel, target_vel):
        current_vel = current_vel * self.robot.MAX_ROT_SPEED_RpS
        target_vel = target_vel * self.robot.MAX_ROT_SPEED_RpS
        dist = abs(current_vel**2 - target_vel**2) / (2 * self.robot.MAX_ANG_ACCELERATION)
        return dist

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
            vel_front = sens.vel[0]

            dist = np.linalg.norm(start_pos - sens.pos)
            if max_dist is not None and dist > max_dist:
                print("\n[drive] max dist reached")
                break

            if front_wall_dist is not None and sens.cur_front_wall_dist < front_wall_dist:
                print("\n[drive] wall reached")
                break

            if (
                left_wall_dist is not None
                and stop_on_wall_hole is not None
                and sens.cur_left_wall_dist > stop_on_wall_hole
            ):
                print("\n[drive] left wall hole reached")
                break

            if (
                right_wall_dist is not None
                and stop_on_wall_hole is not None
                and sens.cur_right_wall_dist > stop_on_wall_hole
            ):
                print("\n[drive] right wall hole reached")
                break

            if smooth_stop_dist > 0:
                stop_dist = smooth_stop_dist
                if max_dist is not None:
                    stop_dist = min(max_dist - dist, stop_dist)
                if front_wall_dist is not None:
                    stop_dist = min(sens.cur_front_wall_dist - front_wall_dist, stop_dist)
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
                left_wall_dist is not None and sens.cur_left_wall_dist < sens.cur_front_wall_dist
            ):
                t = (sens.cur_left_wall_dist - left_wall_dist) / left_wall_dist * 2
            elif (
                right_wall_dist is not None
                and sens.cur_right_wall_dist < sens.cur_front_wall_dist
            ):
                t = -(sens.cur_right_wall_dist - right_wall_dist) / right_wall_dist * 2
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
                msg += f"front_wall: {sens.cur_front_wall_dist:6.3f} "
            if left_wall_dist is not None:
                msg += f"left_wall: {sens.cur_left_wall_dist:6.3f} "
            if right_wall_dist is not None:
                msg += f"right_wall: {sens.cur_right_wall_dist:6.3f} "
            print(msg, end="", flush=True)
            self.robot.send_drive(speed, rot)
            time.sleep(self.CONTROLLER_PERIOD)

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

        sens = self.robot.recv_sensors()
        direction = np.deg2rad(direction)
        if relative:
            direction = sens.angle + direction

        while True:
            sens = self.robot.recv_sensors()

            angle_diff = direction - sens.angle
            angle_diff = np.arctan2(np.sin(angle_diff), np.cos(angle_diff))
            if abs(angle_diff) < self.ANGLE_THRESHOLD:
                print("\n[rotate] target angle reached")
                break

            t = np.clip(angle_diff / self.REGULATE_MAX_ANGLE, -1, 1)
            rot = np.clip(
                t * self.MAX_ROT_SPEED, -self.MAX_ROT_SPEED, self.MAX_ROT_SPEED
            )

            print(f"\r[rotate] th: {sens.angle:6.3f} rot_speed: {rot:6.3f}", end="", flush=True)
            self.robot.send_drive(0, rot)
            time.sleep(self.CONTROLLER_PERIOD)

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
            time.sleep(self.CONTROLLER_PERIOD)

        print("[stop] stopped")
        self.robot.send_drive(0, 0)

    def freeze(self):
        """Return back to position where it was called, decrease inertia"""
        vel_threshold = 0.01
        k = 1
        eps = 0.05

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
            time.sleep(self.CONTROLLER_PERIOD)

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
        max_dist - на какое расстояние проехать
        front_wall_dist - остановиться, если стенка ближе чем это расстояние
        front_wall_smooth_stop_dist, side_wall_smooth_stop_dist - 
            расстояние до конца, на котором начать плавно тормозить
        """
        sens = self.robot.recv_sensors()
        start_pos = sens.pos

        assert left_wall_dist is None or right_wall_dist is None

        while True:
            sens = self.robot.recv_sensors()
            vel_front = sens.vel[0]
            sens.cur_right_wall_dist = self.robot.navigator.get_wall_dist(center_angle=-50, max_angle=10)

            dist = np.linalg.norm(start_pos - sens.pos)

            # if left_wall_dist is not None and sens.cur_left_wall_dist > left_wall_dist + side_wall_smooth_stop_dist:
            #     print("\n[drive] lost left wall")
            #     break
            # if right_wall_dist is not None and sens.cur_right_wall_dist > right_wall_dist + side_wall_smooth_stop_dist:
            #     print("\n[drive] lost right wall")
            #     break

            if left_wall_dist is not None:
                t = (sens.cur_left_wall_dist - left_wall_dist) / side_wall_smooth_stop_dist
            elif right_wall_dist is not None:
                t = (
                    -(sens.cur_right_wall_dist - right_wall_dist)
                    / side_wall_smooth_stop_dist
                )
            else:
                t = 0
            rot = np.clip(t * self.MAX_ROT_SPEED, -self.MAX_ROT_SPEED, self.MAX_ROT_SPEED)

            stop_dist = sens.cur_front_wall_dist - front_wall_dist
            t = np.clip(stop_dist / front_wall_smooth_stop_dist, -1, 1)
            speed = (
                np.sign(max_speed)
                * np.sign(t)
                * np.clip(abs(t) * abs(max_speed), self.MIN_SPEED, abs(max_speed))
            )
            # speed += 1 * (speed - vel_front)

            # speed *= 1 - 0.8 * abs(rot / self.MAX_ROT_SPEED)
            rot = np.clip(
                rot * abs(max_speed) / abs(speed), -self.MAX_ROT_SPEED, self.MAX_ROT_SPEED
            )

            msg = f"\r[drive] "
            msg += f"dist: {dist:6.3f} speed: {speed:6.3f} vel: {vel_front:6.3f} "
            if left_wall_dist is not None or right_wall_dist is not None:
                msg += f"th: {sens.angle:6.3f} rot_speed: {rot:6.3f} "
            if front_wall_dist is not None:
                msg += f"front_wall: {sens.cur_front_wall_dist:6.3f} "
            if left_wall_dist is not None:
                msg += f"left_wall: {sens.cur_left_wall_dist:6.3f} "
            if right_wall_dist is not None:
                msg += f"right_wall: {sens.cur_right_wall_dist:6.3f} "
            print(msg, end="", flush=True)
            # speed = 0.00
            # rot = 1
            self.robot.send_drive(speed, rot)
            time.sleep(self.CONTROLLER_PERIOD)

            self.robot.navigator.display()

        print("[drive] stop")
        self.robot.send_drive(0, 0)

    def maze_forward(self, target_pos, target_speed=None, brake_eps=0.1, max_speed=None, relative=True):
        target_pos = np.asarray(target_pos)
        if target_speed is None:
            target_speed = self.MAZE_TURN_SPEED
        if max_speed is None:
            max_speed = self.MAX_SPEED

        start_sens = self.robot.recv_sensors()
        if relative:
            print(target_pos, start_sens.pos, start_sens.angle)
            target_pos = transform_points(target_pos, start_sens.pos, start_sens.angle)
        direction = normalize(target_pos - start_sens.pos)
        print(target_pos, direction)

        # data = []
        # start_time = time.monotonic()
        # times = []
        is_braking = False
        while True:
            sens = self.robot.recv_sensors()
            distance_left = project_scalar(direction, target_pos - sens.pos)
            vel_fwd = sens.vel[0]

            if distance_left < 0:
                print("\n[maze_forward] target reached")
                break

            if distance_left < self.estimate_brake_distance(vel_fwd, target_speed) + brake_eps:
                is_braking = True

            if is_braking:
                speed = target_speed
            else:
                speed = max_speed

            msg = f"\r[maze_forward] "
            msg += f"dist: {distance_left:6.3f} "
            msg += f"vel_set: {speed:6.3f} "
            msg += f"vel: {vel_fwd:6.3f} "
            print(msg, end="", flush=True)
            self.robot.send_drive(speed, 0)
            # times.append(time.monotonic() - start_time)
            # data.append((distance_left, vel_fwd, speed))
            time.sleep(self.CONTROLLER_PERIOD)
        # plt.close()
        # plt.plot(times, data)
        # plt.savefig("navigator_images/maze_forward.png")
        # plt.close()

        print("[maze_forward] stop")
        self.robot.send_drive(0, 0)

    def maze_turn(self, target_angle, radius=None, speed=None, max_rot_speed=1, brake_eps=0.1, target_rot_vel=0.1, relative=True):
        target_angle = np.deg2rad(target_angle)
        if speed is None:
            speed = self.MAZE_TURN_SPEED
        if radius is None:
            radius = self.MAZE_TURN_RADIUS

        start_sens = self.robot.recv_sensors()
        if relative:
            target_angle = round_angle(target_angle + start_sens.angle)
        angle_dir = np.sign(round_angle(target_angle - start_sens.angle))
        angle_to_center = start_sens.angle + angle_dir * np.pi/2
        center = start_sens.pos + direction_vec(angle_to_center) * radius

        target_rot_vel = target_rot_vel * angle_dir

        # data = []
        # start_time = time.monotonic()
        # times = []
        # prev_th = start_sens.angle
        # prev_time = time.monotonic() - 0.1
        # prev_th_vel = 0
        while True:
            sens = self.robot.recv_sensors()
            dist_to_center = np.linalg.norm(sens.pos - center)
            angle_to_target = round_angle(target_angle - sens.angle) * angle_dir
            th_vel = sens.angle_vel * angle_dir

            # delta_angle = sens.angle - prev_th
            # prev_th = sens.angle

            # time_now = time.monotonic()
            # delta_time = time_now - prev_time
            # prev_time = time_now

            # th_vel = delta_angle / delta_time

            # delta_vel = (th_vel - prev_th_vel) / delta_time
            # prev_th_vel = th_vel 

            if angle_to_target < 0:
                print("\n[maze_turn] target reached")
                break

            if angle_to_target > self.estimate_angular_brake_distance(th_vel, target_rot_vel) + brake_eps:
                rot = max_rot_speed * angle_dir
            else:
                rot = target_rot_vel

            msg = f"\r[maze_forward] "
            msg += f"dist_to_center: {dist_to_center:6.3f} "
            msg += f"angle_to_target: {angle_to_target:6.3f} "
            msg += f"th_vel: {th_vel:6.3f} "
            msg += f"rot: {rot:6.3f} "
            print(msg, end="", flush=True)
            self.robot.send_drive(speed, rot)
            # times.append(time.monotonic() - start_time)
            # data.append((angle_to_target, th_vel, rot))
            time.sleep(self.CONTROLLER_PERIOD)
        # plt.close()
        # plt.plot(times, data)
        # plt.savefig("navigator_images/maze_turn.png")
        # plt.close()
        # print(np.max(data, axis=0))

        print("[maze_turn] stop")
        self.robot.send_drive(0, 0)


    def maze_turnaround(self):
        self.maze_turn(180, speed=0)

