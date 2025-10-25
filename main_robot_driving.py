import threading
import time


class Latest:
    """Потокобезопасное хранилище последнего значения с меткой времени."""
    __slots__ = ("_data", "_ts", "_lock")

    def __init__(self):
        self._data = None
        self._ts = None
        self._lock = threading.Lock()

    def set(self, value):
        """Сохраняет новое значение и метку времени."""
        with self._lock:
            self._data = value
            self._ts = time.monotonic()

    def get(self):
        """Возвращает (value, timestamp) — последнюю запись и время её обновления."""
        with self._lock:
            return self._data, self._ts

    def age(self):
        """Сколько секунд прошло с последнего обновления."""
        with self._lock:
            if self._ts is None:
                return float("inf")
            return time.monotonic() - self._ts


odom_latest  = Latest()
lidar_latest = Latest()


def read_lidar():
    """Должен заранее готовить только необходимые контроллеру данные.
    """
    while True:
        # data = read_lidar_array()
        lidar_latest.set()


def read_odom(robot):
    while True:
        # data = robot.read()
        odom_latest.set()


def controller_loop():
    """
    Здесь бесконечный цикл с логикой езды по координатам.
    Будет работать с классом Robot - получать последние показания 
    датчиков с общих переменных здесь, и отправлять команды на моторы.
    """
    odom_latest.get()
    lidar_latest.get()


if __name__ == "__main__":
    # Здесь выбираем либо реального робота, либо эмуляцию!
    # controller_loop() должен работать с интерфейсом, неважно какой робот - настоящий или нет
    # robot = RobotChassis()
    # robot = EmulatedRobot()
    robot = None
    threading.Thread(target=read_odom,  daemon=True, args=(robot,)).start()
    threading.Thread(target=read_lidar, daemon=True).start()
    controller_loop()
