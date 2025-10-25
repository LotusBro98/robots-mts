import time
from low_level_drive import connect_robot, disconnect_robot, send_cmd


print("connect_robot")
connect_robot()
print("robot connected")
send_cmd(v=1, w=0)
print("Оба мотора едут прямо")
time.sleep(10)
send_cmd(v=0, w=0)
print("Оба остановлены")
time.sleep(5)
send_cmd(v=0.5, w=1)
print("Широкий поворот налево")
time.sleep(10)
send_cmd(v=0, w=0)
print("Оба остановлены")
time.sleep(5)
disconnect_robot()
print("disconnect_robot")
