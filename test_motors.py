import time

from chassis import RobotChassis
robot = RobotChassis()

print("connect_robot")
robot.connect()
robot.start_sensor_capture()
print("robot connected")
time.sleep(5)
robot.send_drive(v=1, w=0)
print("Оба мотора едут прямо")
time.sleep(10)
robot.send_drive(v=0, w=0)
print("Оба остановлены")
time.sleep(5)
robot.send_drive(v=0.5, w=1)
print("Широкий поворот налево")
time.sleep(10)
robot.send_drive(v=0, w=0)
print("Оба остановлены")
time.sleep(5)
robot.disconnect()
print("disconnect_robot")
