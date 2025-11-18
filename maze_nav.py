from chassis import RobotChassis
from driver import Driver
from emulation.emulated_robot import EmulatedRobot

def main():
    robot = EmulatedRobot(demo_render_navigator=False, file_rendering_navigator=True)  # Flags: demo_render_navigator, file_rendering_navigator
    # robot = EmulatedRobot(demo_render_navigator=True, file_rendering_navigator=False)  # Flags: demo_render_navigator, file_rendering_navigator
    robot.connect()
    driver = Driver(robot)
    driver.MAX_SPEED = 1
    # driver.MAX_ROT_SPEED = 0.5
    robot.navigator.goal = (7.5, 7.5)

    # robot = RobotChassis()
    # robot.connect()
    # driver = Driver(robot)
    # driver.MAX_SPEED = 1.0
    # driver.MAX_ROT_SPEED = 0.8

    # Ограничение линейной скорости *на поворотах*
    driver.MAX_SPEED_ON_TURN = 1.0
    # Коэффициент дифференциальной части для регулятора поворота
    driver.SIDE_WALL_STABILIZE_COEFF = 0

    try:
        driver.drive_trajectory()
    finally:
        driver.robot.send_drive(0, 0)
        robot.navigator.stop()
        robot.disconnect()
        # robot.navigator.stop_rendering()

if __name__ == "__main__":
    main()
