import json
import time
import traceback
import serial
import threading

class RobotChassis:
    def __init__(self, port: str = "/dev/ttyACM1"):
        self.port = port
        
    def connect(self):
        self.ser = serial.Serial(self.port, baudrate=115200, timeout=1)
        self.start_sensor_capture()
    
    def disconnect(self):
        self.do_capture_sensors = False
        self.sensors_thread.join()
        self.ser.close()

    def send_command(self, **kwargs):
        cmd_json = json.dumps(kwargs, separators=(',', ':')) + "\r\n"
        print(cmd_json)
        self.ser.write(cmd_json.encode())

    def start_sensor_capture(self):
        self.do_capture_sensors = True
        self.sensors_thread = threading.Thread(target=self._capture_wheel_sensors)
        self.sensors_thread.start()

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
                print("Error parsing message frim chassis:", e)
                continue

            try:
                self.sensors_callback(msg)
            except:
                traceback.print_exc()
                continue

    def sensors_callback(self, msg):
        print("Chassis sensors: ", msg) # {"T":1001,"M1":0,"M2":0,"M3":0,"M4":0,"odl":3247,"odr":8920,"v":963}

def main():
    robot = RobotChassis("/dev/ttyACM1")
    robot.connect()
    time.sleep(10)
    # robot.send_command(T=131, cmd=1)
    robot.disconnect()

if __name__ == "__main__":
    main()
