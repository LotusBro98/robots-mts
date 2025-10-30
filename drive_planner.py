from drive_planning.corridor_distance import find_openings_left_right
from driver import Driver
from navigator import Navigator


class RobotDrivePlanner:
    DOOR_SIZE = 0.5

    def __init__(self, driver: Driver):
        self.driver = driver
        self.navigator: Navigator = driver.robot.navigator
        self.robot = driver.robot

        self.DRIVES = [
            (self.driver.maze_forward, (0.25,)),
            (self.driver.maze_turn, (90,)),
            (self.driver.maze_turn, (-90,)),
            (self.driver.maze_turn, (90,)),
            (self.driver.maze_turn, (-90,)),
            (self.driver.maze_forward, (0.5,)),
            (self.driver.maze_turn, (-90,)),
            (self.driver.maze_forward, (0.5,)),
            (self.driver.maze_turn, (90,)),
            (self.driver.maze_forward, (3,)),
            (self.driver.maze_turnaround, ()),
            (self.driver.maze_forward, (3,)),
        ]
        

    """
    использует Driver для вызова манёвров, Robot.recv_sensors для получения данных датчиков, Robot.navigator.
    Главная логика:
    - если точно тупик, разворачиваемся
    - если сейчас есть поворот направо -> направо
    - если впереди коридор, едем до поворота или на макс расстояние пока не станет видно дальше
    - если сейчас есть поворот налево -> налево
    """
    cnt = 0
    def calculate_next_drive(self):
        """
        Рассчитывает следующий манёвр для текущего положения робота.
        """

        result_wall_openings = find_openings_left_right(self.navigator)
        self.navigator.set_external_openings(result_wall_openings, inlier_tol=0.06)

        op_right = result_wall_openings["right"]
        op_left = result_wall_openings["left"]
        op_nearest = result_wall_openings["nearest"]

        dead_end = not op_left.found and not op_right.found
        if dead_end:
            return self.driver.maze_turnaround, (), {}
        elif op_nearest.distance > self.DOOR_SIZE / 3:
            return self.driver.maze_forward, (op_nearest.distance,), {}
        elif op_right.found:
            return self.driver.maze_turn, (-90,), {"target_pos": op_right.world_gap_center}
        elif op_left.found:
            return self.driver.maze_turn, (90,), {"target_pos": op_left.world_gap_center}
        else:
            print("Don't know what to do")
            return self.driver.maze_forward, (self.DOOR_SIZE,), {}

        self._calculate_distance_to_next_turn_in_corridor()
        ret = self.DRIVES[self.cnt] + ({},)
        self.cnt += 1
        return ret
    
    def main_loop(self):
        while True:
            next_drive, args, kwargs = self.calculate_next_drive()
            next_drive(*args, **kwargs)
            if self.cnt >= len(self.DRIVES):
                break
