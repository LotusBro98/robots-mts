from drive_planning.corridor_distance import find_openings_left_right
from driver import Driver
from navigator import Navigator


class RobotDrivePlanner:
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

    def _calculate_distance_to_next_turn_in_corridor(self) -> tuple[str, float]:
        """
        Возвращает расстояние в метрах до следующего поворота, и само направление - направо/налево
        Используем из navigator:
            pos: np.ndarray of 2 elem
            angle: float
            points: np.ndarray
        """
        points_xy_world = self.navigator.points
        robot_xy = self.navigator.pos
        robot_heading_rad = self.navigator.angle
        result_wall_openings = find_openings_left_right(points_xy_world, robot_xy, robot_heading_rad,
                                    min_open=0.20, inlier_tol=0.06, dx=0.05)
        # self.navigator.set_external_openings(result_wall_openings, inlier_tol=0.06)
        if result_wall_openings and result_wall_openings.get("nearest") and result_wall_openings["nearest"].side:
            result_distance_for_opening = result_wall_openings["nearest"]
            return result_distance_for_opening.side, result_distance_for_opening.distance
