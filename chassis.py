import json
import time
import traceback
import serial
import threading
import numpy as np

class RobotChassis:


    def __init__(self, port: str = "/dev/ttyACM1"):
        self.port = port
        self.sensors_thread = None
        
    def connect(self):
        self.ser = serial.Serial(self.port, baudrate=115200, timeout=1)
    
    def disconnect(self):
        self.stop_sensors_capture()
        self.ser.close()

    def send_command(self, **kwargs):
        cmd_json = json.dumps(kwargs, separators=(',', ':'))
        print("Cmd to chassis:", cmd_json)
        self.ser.write((cmd_json + "\r\n").encode())

    def start_sensor_capture(self):
        self.do_capture_sensors = True
        self.last_msg = None
        self.pos = np.array([0.0, 0.0], dtype=np.float32)
        self.angle = 0
        self.send_command(T=131, cmd=1)
        self.sensors_thread = threading.Thread(target=self._capture_wheel_sensors, daemon=True)
        self.sensors_thread.start()

    def stop_sensors_capture(self):
        self.do_capture_sensors = False
        if self.sensors_thread is not None:
            self.sensors_thread.join()
            self.sensors_thread = None
            self.send_command(T=131, cmd=0)

    def _capture_wheel_sensors(self):
        print("Started wheel capture")
        while self.do_capture_sensors:
            try:
                msg = self.ser.read_until(b"\r\n").strip(b"\r\n")
            except:
                traceback.print_exc()
                break

            try:
                msg = json.loads(msg.decode())
            except Exception as e:
                print("Error parsing message from chassis:", e, msg)
                continue

            try:
                self.sensors_callback(msg)
            except:
                traceback.print_exc()
                continue

    def calc_odometry(self, msg, last_msg):
        delta_left = msg["odl"] - last_msg["odl"]
        delta_right = msg["odr"] - last_msg["odr"]

        linear_delta = 0.5 * (delta_left + delta_right) * 0.01 # original unit is cm
        angular_delta = 0.5 * (delta_right - delta_left)



    def sensors_callback(self, msg):
        print("Chassis sensors: ", msg) # {"T":1001,"M1":0,"M2":0,"M3":0,"M4":0,"odl":3247,"odr":8920,"v":963}
        if self.last_msg is None:
            self.last_msg = msg
            return
        
        self.calc_odometry(msg, self.last_msg)
        self.last_msg = msg

def main():
    robot = RobotChassis("/dev/ttyACM1")
    robot.connect()
    robot.start_sensor_capture()
    time.sleep(1000)
    robot.stop_sensors_capture()
    robot.disconnect()

if __name__ == "__main__":
    main()
