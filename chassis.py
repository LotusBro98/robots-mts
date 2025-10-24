import json
import traceback
import serial
import threading

class RobotChassis:
    def __init__(self, port: str = "/dev/ttyACM1"):
        self.port = port
        self.sensors_thread = threading.Thread
        
    def connect(self):
        self.ser = serial.Serial(self.port, baudrate=115200, timeout=1)
    
    def disconnect(self):
        self.ser.close()

    def send_command(self, **kwargs):
        cmd_json = json.dumps(kwargs, separators=(',', ':')) + "\r\n"
        print(cmd_json)
        self.ser.write(cmd_json.encode())
        # head = self.ser.read_until(b"{")
        # response = b"{" + self.ser.read_until(b"\r\n")
        # print(response.decode())
        # response = json.loads(response)
        # return response

    def _capture_wheel_sensors(self):
        while True:
            try:
                msg = self.ser.read_until(b"\r\n").strip(b"\r\n")
            except:
                traceback.print_exc()
                break

            try:
                msg = json.loads(msg.decode())
            except:
                traceback.print_exc()
            print(msg) # b'{"T":1001,"M1":0,"M2":0,"M3":0,"M4":0,"odl":3247,"odr":8920,"v":963}'
        # res = self.send_command(T=130)
        print(res)

    def sensors_callback(self, msg):
        print("Chassis sensors: ", msg)

def main():
    robot = RobotChassis("/dev/ttyACM1")
    robot.connect()
    robot.capture_wheel_sensors()
    # robot.send_command(T=131, cmd=1)
    robot.disconnect()

if __name__ == "__main__":
    main()
