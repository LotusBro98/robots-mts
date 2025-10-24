import json
import serial

class RobotChassis:
    def __init__(self, port: str = "/dev/ttyACM1"):
        self.port = port
        
    def connect(self):
        self.ser = serial.Serial(self.port, baudrate=115200, timeout=1)
    
    def disconnect(self):
        self.ser.close()

    def send_command(self, **kwargs):
        cmd_json = json.dumps(kwargs, separators=(',', ':'))
        print(cmd_json)
        self.ser.write(cmd_json.encode())
        # head = self.ser.read_until(b"{")
        # response = b"{" + self.ser.read_until(b"\r\n")
        # print(response.decode())
        # response = json.loads(response)
        # return response

    def capture_wheel_sensors(self):
        while True:
            msg = self.ser.read_until(b"\r\n").strip(b"\r\n")
            print(msg)
        # res = self.send_command(T=130)
        print(res)


def main():
    robot = RobotChassis("/dev/ttyACM1")
    robot.connect()
    # robot.capture_wheel_sensors()
    robot.send_command(T=1, L=10, R=10)
    robot.disconnect()

if __name__ == "__main__":
    main()
