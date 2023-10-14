import serial
import time
import math
from utils.mpManager import MPManager

class Serializer:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        self.ser.reset_input_buffer()
        self.data = SerialData()

    def read(self):
        line = self.ser.readline().decode('utf-8', errors="ignore").rstrip()
        # print("serial read: " + str(line))
        self.decode_string(line)
    
    def write(self):
        self.ser.write(self.encode_string())
        # print("serial write: " + str(self.encode_string()))

    def decode_string(self,input_string):
        # Extracting numbers from the input_string using string manipulation
        # Line Format: "Wheels: [wl, wr]"
        if not input_string.startswith("Nano"):
            return
    
        try:
            start_idx = input_string.index("[") + 1
            end_idx = input_string.index("]")
            wl_current, wr_current, wl_goal, wr_goal = [float(num) for num in input_string[start_idx:end_idx].split(',')]
        except ValueError:
            raise ValueError("Invalid input format")

        # Creating a SerialData object with the extracted numbers
        self.data.update(wl_current=wl_current, wr_current=wr_current, wl_goal=wl_goal, wr_goal=wr_goal)
        
    
    def encode_string(self):
        return f"Pi: [{self.data.wl_current},{self.data.wr_current},{self.data.wl_goal},{self.data.wr_goal}]".encode("utf-8")

class SerialData:
    """
    Data for Serial Connection between Arduino and PI

    All Velocities in RPM
    """
    def __init__(self, wl_current=0, wr_current=0, wl_goal=0, wr_goal=0):
        self.update(wl_current, wr_current, wl_goal, wr_goal)

    def update(self, wl_current=None, wr_current=None, wl_goal=None, wr_goal=None):
        if wl_current is not None:
            self.wl_current = wl_current
        if wr_current is not None:
            self.wr_current = wr_current
        if wl_goal is not None:
            self.wl_goal = wl_goal
        if wr_goal is not None:
            self.wr_goal = wr_goal

def serializer_loop(manager_mp: MPManager):
    serializer = Serializer()
    while True:
        
        # Read Data
        # serializer.read()
        wl_goal_rpm = manager_mp.wl_goal_value * 60 / (2 * math.pi)
        wr_goal_rpm = manager_mp.wr_goal_value * 60 / (2 * math.pi)
        # print(wl_goal_rpm)
        # print(wr_goal_rpm)
        serializer.data.update(wl_goal=wl_goal_rpm, wr_goal=wr_goal_rpm)
        serializer.write()
        time.sleep(0.01)
        serializer.read()

        # print("loop 2")
        manager_mp.current_wl = serializer.data.wl_current
        manager_mp.current_wr = serializer.data.wr_current
        
        #print("loop 2")