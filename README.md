# Show lidar online:
export MPLBACKEND=WebAgg
export LIDAR_PORT="/dev/ttyUSB0"
python demo_show_lidar_online.py --port /dev/ttyUSB0


Использование navigator.py:

1. Окно в отдельном потоке, 15 FPS:
```
nav = Navigator(show_demo=True, render_mode="window", render_fps=15.0)
```

2. Без окна, запись кадров в PNG-папку frames/ с 5 FPS:
```
nav = Navigator(show_demo=True, render_mode="file", render_fps=5.0, render_out_dir="frames")
```

3. Полностью без рендера (только расчёты):
```
nav = Navigator(show_demo=False)  # render_mode автоматически "off"
```

Корректное завершение при выходе:
nav.stop_rendering()
