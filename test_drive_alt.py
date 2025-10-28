import time
from driver import Driver
from emulation.emulated_robot import EmulatedRobot

robot = EmulatedRobot(demo_render_navigator=False, file_rendering_navigator=False)  # Flags: demo_render_navigator, file_rendering_navigator
robot.connect()
driver = Driver(robot)
driver.update_ref_angle()

try:
    driver.rotate(180, relative=False)
    driver.stop()
    time.sleep(3)
    driver.rotate(-180, relative=False)
    driver.stop()
    time.sleep(5)
finally:
    robot.disconnect()
    robot.navigator.stop_rendering()
