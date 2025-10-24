import json
import serial

class RobotChassis:
    def __init__(self, port: str):
        self.port = port
        
    def connect(self):
        self.ser = serial.Serial(self.port, baudrate=9600, timeout=1)
    
    def disconnect(self):
        self.ser.close()

    def send_command(self, **kwargs):
        cmd_json = json.dumps(kwargs)
        self.ser.write(cmd_json)
        response = self.ser.read_all()
        response = json.loads(response)
        return response

    def get_wheel_sensors(self):
        res = self.send_command(T=130)
        print(res)


def main():
    robot = RobotChassis("/dev/usb0")
    robot.connect()
    robot.get_wheel_sensors()

if __name__ == "__main__":
    main()
