#!/usr/bin/env python3
# teleop_wasd.py — cross-platform (Linux/macOS/Windows)

import sys, time, platform

from chassis import RobotChassis
from driver import Driver
from emulation.emulated_robot import EmulatedRobot

HZ = 10.0
V_MAX = 1.0
W_MAX = 1.0
DV_STEP = 0.05
DW_STEP = 0.05
ACCEL_MULT_SHIFT = 2.0
ROBOT_CLASS = RobotChassis
ROBOT_CLASS = EmulatedRobot


HELP_TEXT = """
W/S: линейная ↑/↓   A/D: угловая ←/→
Стрелки тоже работают
Space: стоп   X: экстренный стоп
R/F: шаг линейной +/-
T/G: шаг угловой +/-
Q или Esc: выход
(Заглавные WASD = как будто с Shift: увеличенный шаг)
"""

IS_WINDOWS = platform.system().lower().startswith("win")

if IS_WINDOWS:
    # ── Ввод с клавиатуры под Windows (msvcrt) ────────────────────────────────
    import msvcrt

    def get_key_nonblock():
        """Возвращает один «символьный» ключ или спец-слово ('UP','DOWN','LEFT','RIGHT','ESC'), либо None."""
        if not msvcrt.kbhit():
            return None
        ch = msvcrt.getwch()  # Unicode
        # Спец-клавиши: возвращают префикс \x00 или \xe0, затем собственно код
        if ch in ('\x00', '\xe0'):
            code = msvcrt.getwch()
            return {'H': 'UP', 'P': 'DOWN', 'K': 'LEFT', 'M': 'RIGHT'}.get(code)
        if ch == '\x1b':
            return 'ESC'
        return ch

    class RawMode:
        def __enter__(self):  # raw режим не нужен — консоль и так без буфера по Enter
            # Скрыть курсор (Windows 10+ обычно поддерживает ANSI; если нет — просто проигнорируется)
            sys.stdout.write("\x1b[?25l"); sys.stdout.flush()
        def __exit__(self, exc_type, exc, tb):
            sys.stdout.write("\x1b[?25h\n"); sys.stdout.flush()

else:
    # ── Ввод с клавиатуры под POSIX (termios/tty) ─────────────────────────────
    import termios, tty, select

    def _kbhit(timeout=0.0):
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return bool(r)

    def get_key_nonblock():
        if not _kbhit(0.0):
            return None
        ch = sys.stdin.read(1)
        if ch == '\x1b':  # ESC или ESC-последовательность стрелок
            if _kbhit(0.001) and sys.stdin.read(1) == '[' and _kbhit(0.001):
                ch2 = sys.stdin.read(1)
                return {'A': 'UP', 'B': 'DOWN', 'C': 'RIGHT', 'D': 'LEFT'}.get(ch2, 'ESC')
            return 'ESC'
        return ch

    class RawMode:
        def __enter__(self):
            self._old = termios.tcgetattr(sys.stdin.fileno())
            tty.setraw(sys.stdin.fileno())
            sys.stdout.write("\x1b[?25l"); sys.stdout.flush()
        def __exit__(self, exc_type, exc, tb):
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._old)
            sys.stdout.write("\x1b[?25h\n"); sys.stdout.flush()

def clamp(x, lo, hi): return max(lo, min(hi, x))

def print_speeds(v, w):
    sys.stdout.write(f"\rSpeed  v={v:+.3f}  w={w:+.3f}")
    sys.stdout.flush()

def main():
    print(HELP_TEXT)
    print("Старт. Нажмите Q или Esc для выхода.", flush=True)

    v, w = 0.0, 0.0
    dv, dw = DV_STEP, DW_STEP
    t_period = 1.0 / HZ
    last_send = 0.0

    robot = ROBOT_CLASS()
    robot.connect()
    driver = Driver(robot)

    try:
        with RawMode():
            while True:
                frame_start = time.time()

                # считываем все доступные кнопки за кадр
                while True:
                    key = get_key_nonblock()
                    if key is None:
                        break

                    # «Увеличенный шаг» если заглавная буква (похоже на удержание Shift)
                    mult = ACCEL_MULT_SHIFT if isinstance(key, str) and key.isalpha() and key.isupper() else 1.0

                    if key in ('q', 'Q', 'ESC'):
                        v, w = 0.0, 0.0
                        robot.send_drive(v, w)
                        print("\nВыход.")
                        return

                    elif key == ' ':
                        v, w = 0.0, 0.0
                    elif key in ('x', 'X'):
                        v, w = 0.0, 0.0
                        robot.send_drive(v, w)

                    elif key in ('w', 'W', 'UP'):
                        v = clamp(v + dv * mult, -V_MAX, V_MAX)
                    elif key in ('s', 'S', 'DOWN'):
                        v = clamp(v - dv * mult, -V_MAX, V_MAX)
                    elif key in ('a', 'A', 'LEFT'):
                        w = clamp(w + dw * mult, -W_MAX, W_MAX)
                    elif key in ('d', 'D', 'RIGHT'):
                        w = clamp(w - dw * mult, -W_MAX, W_MAX)

                    elif key in ('r', 'R'):
                        dv = min(dv * 1.25, V_MAX); print(f"\nШаг линейной: {dv:.3f}")
                    elif key in ('f', 'F'):
                        dv = max(dv / 1.25, 0.001); print(f"\nШаг линейной: {dv:.3f}")
                    elif key in ('t', 'T'):
                        dw = min(dw * 1.25, W_MAX); print(f"\nШаг угловой: {dw:.3f}")
                    elif key in ('g', 'G'):
                        dw = max(dw / 1.25, 0.001); print(f"\nШаг угловой: {dw:.3f}")
                    # прочие игнорируем

                # периодическая отправка текущих v,w
                now = time.time()
                if now - last_send >= t_period:
                    robot.send_drive(v, w)
                    print_speeds(v, w)
                    last_send = now

                # держим частоту
                sleep_left = t_period - (time.time() - frame_start)
                if sleep_left > 0:
                    time.sleep(sleep_left)

    except KeyboardInterrupt:
        pass
    finally:
        try: robot.send_drive(0.0, 0.0)
        except Exception: pass

if __name__ == "__main__":
    main()
