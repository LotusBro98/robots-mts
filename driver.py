import time
from matplotlib import pyplot as plt
import numpy as np

from navigator_utils import direction_vec, normalize, project_scalar, round_angle, transform_points, vec_angle
from robot_base import Robot


class Driver:
    MAX_ROT_SPEED = 1.0
    REGULATE_MAX_ANGLE = np.deg2rad(30)
    MIN_SPEED = 0.01
    MAX_SPEED = 1.0
    SMOOTH_STOP_DIST = 0.0
    ANGLE_THRESHOLD = np.deg2rad(5)
    CONTROLLER_PERIOD = 0.05
    MAX_SPEED_ON_TURN = 0.15
    MAZE_TURN_RADIUS = 0.25
    MAZE_RIGHT_WALL_DIST = 0.235
    WALL_ROT_COEFF = 2.0
    WALL_ANGLE_COEFF = 3.0
    WALL_MAX_DECLINE = np.deg2rad(20)
    BRAKE_EPS_LINEAR = 0.1
    BRAKE_EPS_ANGULAR = 0.1
    SIDE_WALL_STABILIZE_COEFF = 10

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

        print()
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

    prev_rwd = None
    prev_lwd = None
    def drive_maze(
        self,
        front_wall_dist,
        front_wall_smooth_stop_dist,
        side_wall_smooth_stop_dist,
        max_speed=None,
        left_wall_dist=None,
        right_wall_dist=None,
    ):
        if max_speed is None:
            max_speed = self.MAX_SPEED

        sens = self.robot.recv_sensors()

        assert left_wall_dist is None or right_wall_dist is None

        while True:
            sens = self.robot.recv_sensors()
            vel_front = sens.vel[0]

            # Angular speed regulator
            if left_wall_dist is not None:
                if max(sens.cur_front_wall_dist, sens.cur_left_wall_dist) < left_wall_dist * 1.5:
                    t = -1
                    self.prev_lwd = None
                elif sens.cur_left_wall_dist > left_wall_dist * 1.5:
                    t = 1
                    self.prev_lwd = None
                else:
                    if self.prev_lwd is None:
                        self.prev_lwd = left_wall_dist
                    lwd_speed = (sens.cur_left_wall_dist - self.prev_lwd) / self.CONTROLLER_PERIOD
                    self.prev_lwd = sens.cur_left_wall_dist

                    t = (sens.cur_left_wall_dist - left_wall_dist) / side_wall_smooth_stop_dist
                    t += lwd_speed * self.SIDE_WALL_STABILIZE_COEFF
            elif right_wall_dist is not None:
                if max(sens.cur_front_wall_dist, sens.cur_right_wall_dist) < right_wall_dist * 1.5:
                    t = 1
                    self.prev_rwd = None
                elif sens.cur_right_wall_dist > right_wall_dist * 1.5:
                    t = -1
                    self.prev_rwd = None
                else:
                    if self.prev_rwd is None:
                        self.prev_rwd = right_wall_dist
                    rwd_speed = (sens.cur_right_wall_dist - self.prev_rwd) / self.CONTROLLER_PERIOD
                    self.prev_rwd = sens.cur_right_wall_dist

                    t = -(sens.cur_right_wall_dist - right_wall_dist) / side_wall_smooth_stop_dist
                    t -= rwd_speed * self.SIDE_WALL_STABILIZE_COEFF
            else:
                t = 0
            t_rot = t
            rot = t * self.MAX_ROT_SPEED
            rot = np.clip(rot, -self.MAX_ROT_SPEED, self.MAX_ROT_SPEED)

            # Linear speed regulator
            stop_dist = sens.cur_front_wall_dist - front_wall_dist
            t = np.clip(stop_dist / front_wall_smooth_stop_dist, -1, 1)
            t += 0.2
            speed = (
                np.sign(max_speed)
                * np.sign(t)
                * np.clip(abs(t) * abs(max_speed), self.MIN_SPEED, abs(max_speed))
            )
            # Speed clamp on turn
            if abs(rot) > 0.5 * self.MAX_ROT_SPEED:
                speed = np.clip(speed, None, self.MAX_SPEED_ON_TURN)

            msg = f"\r[drive] "
            msg += f"speed: {speed:6.3f} "
            msg += f"vel: {vel_front:6.3f} "
            msg += f"t_rot: {t_rot:6.3f} "
            msg += f"th_vel: {sens.angle_vel:6.3f} "
            if left_wall_dist is not None or right_wall_dist is not None:
                msg += f"th: {sens.angle:6.3f} rot_speed: {rot:6.3f} "
            if front_wall_dist is not None:
                msg += f"front_wall: {sens.cur_front_wall_dist:6.3f} "
            if left_wall_dist is not None:
                msg += f"left_wall: {sens.cur_left_wall_dist:6.3f} "
            if right_wall_dist is not None:
                msg += f"right_wall: {sens.cur_right_wall_dist:6.3f} "
            # print(msg, end="", flush=True)
            self.robot.send_drive(speed, rot)
            time.sleep(self.CONTROLLER_PERIOD)

        print("[drive] stop")
        self.robot.send_drive(0, 0)

    def maze_forward(self, target_dist, target_speed=None, brake_eps=None, right_wall_dist=None, max_speed=None, relative=True):
        target_pos = (target_dist, 0)
        target_pos = np.asarray(target_pos)
        if target_speed is None:
            target_speed = self.MAX_SPEED_ON_TURN
        if max_speed is None:
            max_speed = self.MAX_SPEED
        if right_wall_dist is None:
            right_wall_dist = self.MAZE_RIGHT_WALL_DIST
        if brake_eps is None:
            brake_eps = self.BRAKE_EPS_LINEAR

        start_sens = self.robot.recv_sensors()
        if relative:
            target_pos = transform_points(target_pos, start_sens.pos, start_sens.angle)
        direction = normalize(target_pos - start_sens.pos)

        # data = []
        # start_time = time.monotonic()
        # times = []
        is_braking = False
        print()
        while True:
            sens = self.robot.recv_sensors()
            distance_left = project_scalar(direction, target_pos - sens.pos)
            wall_dist, wall_angle = self.robot.navigator.get_wall_dist_and_angle(center_angle=-90, max_angle=60, max_dist=0.5)
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

            if wall_angle is None:
                tgt_rel_angle = 0
            elif is_braking:
                tgt_rel_angle = wall_angle
            else:
                tgt_rel_angle = wall_angle
                tgt_rel_angle += np.clip((right_wall_dist - wall_dist) * self.WALL_ANGLE_COEFF, -self.WALL_MAX_DECLINE, self.WALL_MAX_DECLINE)

            rot = np.clip(tgt_rel_angle * self.WALL_ROT_COEFF, -1, 1) * self.MAX_ROT_SPEED

            msg = f"\r[maze_forward] "
            msg += f"dist: {distance_left:6.3f} "
            msg += f"vel_set: {speed:6.3f} "
            msg += f"vel: {vel_fwd:6.3f} "
            msg += f"rot: {rot:6.3f} "
            if wall_dist is None:
                msg += f"wall_dist: {None}   "
                msg += f"wall_angle: {None}   "
            else:
                msg += f"wall_dist: {wall_dist or -1:6.3f} "
                msg += f"wall_angle: {wall_angle or -1:6.3f} "
            print(msg, end="", flush=True)
            self.robot.send_drive(speed, rot)
            # times.append(time.monotonic() - start_time)
            # data.append((distance_left, vel_fwd, speed))
            time.sleep(self.CONTROLLER_PERIOD)
        # plt.close()
        # plt.plot(times, data)
        # plt.savefig("navigator_images/maze_forward.png")
        # plt.close()

        print("[maze_forward] stop")
        self.robot.send_drive(0, 0)

    def maze_turn(self, target_angle, target_pos=None, radius=None, max_speed=None, max_rot_speed=None, brake_eps=None, target_rot_vel=0.1, relative=True, round_to_90=True):
        target_angle = np.deg2rad(target_angle)
        if max_speed is None:
            max_speed = self.MAX_SPEED_ON_TURN
        if radius is None:
            radius = self.MAZE_TURN_RADIUS
        if brake_eps is None:
            brake_eps = self.BRAKE_EPS_ANGULAR
        if max_rot_speed is None:
            max_rot_speed = self.MAX_ROT_SPEED

        start_sens = self.robot.recv_sensors()
        if relative:
            target_angle = round_angle(target_angle + start_sens.angle)
        if round_to_90:
            target_angle = round(target_angle / (np.pi/2)) * (np.pi/2)
        
        if target_pos is not None:
            target_pos_rel = self.robot.navigator.project_to_robot(target_pos)

        full_angle = abs(round_angle(target_angle - start_sens.angle))
        angle_dir = np.sign(round_angle(target_angle - start_sens.angle))
        angle_to_center = target_angle + angle_dir * np.pi/2
        center = target_pos + direction_vec(angle_to_center) * radius

        target_rot_vel = target_rot_vel * angle_dir

        print()
        while True:
            sens = self.robot.recv_sensors()
            angle_to_target = round_angle(target_angle - sens.angle) * angle_dir
            angle_to_center = round_angle(vec_angle(target_pos - center) - vec_angle(sens.pos - center)) * angle_dir
            cur_radius = np.linalg.norm(sens.pos - center)
            # tgt_radius = start_radius + angle_to_center / full_angle * (end_radius - start_radius)
            tgt_radius = radius
            th_vel = sens.angle_vel * angle_dir

            if angle_to_target < 0:
                print("\n[maze_turn] target reached")
                break

            # target_angle = angle_to_center

            speed = max_speed * (1 + (angle_to_center - angle_to_target) * 1.0)

            if angle_to_target > self.estimate_angular_brake_distance(th_vel, target_rot_vel) + brake_eps:
                rot = max_rot_speed * angle_dir * (1 + (cur_radius - tgt_radius) * 1.0)
            else:
                rot = target_rot_vel

            msg = f"\r[maze_turn] "
            msg += f"cur_radius: {cur_radius:6.3f} "
            msg += f"tgt_radius: {tgt_radius:6.3f} "
            msg += f"angle_to_target: {angle_to_target:6.3f} "
            msg += f"angle_to_center: {angle_to_center:6.3f} "
            msg += f"th_vel: {th_vel:6.3f} "
            msg += f"rot: {rot:6.3f} "
            print(msg, end="", flush=True)
            self.robot.send_drive(speed, rot)
            time.sleep(self.CONTROLLER_PERIOD)

        print("[maze_turn] stop")
        self.robot.send_drive(0, 0)


    def maze_turnaround(self, target_rot_vel=0.1, brake_eps=None, max_rot_speed=None):
        start_sens = self.robot.recv_sensors()
        target_angle = np.deg2rad(180) + start_sens.angle
        if brake_eps is None:
            brake_eps = self.BRAKE_EPS_ANGULAR
        if max_rot_speed is None:
            max_rot_speed = self.MAX_ROT_SPEED
        angle_dir = np.sign(round_angle(target_angle - start_sens.angle))

        print()
        while True:
            sens = self.robot.recv_sensors()
            angle_to_target = round_angle(target_angle - sens.angle) * angle_dir
            th_vel = sens.angle_vel

            if abs(angle_to_target) < np.pi / 2 and angle_to_target < 0:
                print("\n[maze_turnaround] target reached")
                break

            if abs(angle_to_target) > self.estimate_angular_brake_distance(th_vel, target_rot_vel) + brake_eps:
                rot = max_rot_speed * angle_dir
            else:
                rot = target_rot_vel

            msg = f"\r[maze_turnaround] "
            msg += f"angle_to_target: {angle_to_target:6.3f} "
            msg += f"th_vel: {th_vel:6.3f} "
            msg += f"rot: {rot:6.3f} "
            print(msg, end="", flush=True)
            self.robot.send_drive(0, rot)
            time.sleep(self.CONTROLLER_PERIOD)

        print("[maze_turnaround] stop")
        self.robot.send_drive(0, 0)

