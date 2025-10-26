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
    # TODO: здесь же нужно рассчитывать скорости робота! odom_x, odom_y, odom_th, vx, vy, vth
    while True:
        # data = robot.read()
        odom_latest.set()


def controller_loop():
    """
    Здесь бесконечный цикл с логикой езды по координатам.
    Будет работать с классом Robot - получать последние показания 
    датчиков с общих переменных здесь, и отправлять команды на моторы.

    Текущая такая функция - drive(), перенесётся скоро
    """
    odom_latest.get()
    lidar_latest.get()


if __name__ == "__main__":
    # Здесь выбираем либо реального робота, либо эмуляцию!
    # controller_loop() должен работать с интерфейсом, неважно какой робот - настоящий или нет
    # robot = RobotChassis()
    # robot = EmulatedRobot()

    # В каждом robot будет возможность вытащить те самые odom_latest и lidar_latest.
    # треды для чтения датчиков будут создаваться внутри самого robot, по функции _start_capture()?
    robot = None
    threading.Thread(target=read_odom,  daemon=True, args=(robot,)).start()
    threading.Thread(target=read_lidar, daemon=True).start()
    controller_loop()  # Здесь частоту будем ставить меньше, чем считывание датчиков
    # main_func() будет вызывать класс Driver с разными режимами езды с циклами. 
    # Нужно ограничение частоты, связанное с инертностью робота.
    # Например режим езды: "развернуться", "ехать по стенке пока не увидим дырку в стене"

    # Навигатор - должен работать со своей частотой в отдельном thread. он не зависит от робота!
    # Навигатор: карта + собственные координаты
    # TODO: сделать, чтобы он обновлялся с частотой одометрии.
    # Навигатор нет смысла обновлять чаще, чем одометрию
    # Обновление лидара на все 360 градусов должно триггерить обновление карты.
