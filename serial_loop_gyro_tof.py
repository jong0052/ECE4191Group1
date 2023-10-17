import serial
import numpy as np
import time
import math

# Define the Class Serializer GT
class Serializer_GT:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
        self.ser.reset_input_buffer()
        self.ang_data = AngleData()
        self.sensor_data = SensorData()
        self.loc_data = LocalizationData()

    # Class Function to Activate the Measurement Tool To start calibration.
    def activate(self):
        encoded_string = f"[201]".encode("utf-8")
        self.ser.write(encoded_string)
        # print("Calibrating! Press ENTER to Proceed.")
    
    # Terminate The pySerial Connection
    def close(self):
        self.ser.close()

    def read_ang(self):
        for i in range(0,10,1):
            encoded_string = f"[221]".encode("utf-8")
            self.ser.write(encoded_string)
            line = self.ser.readline().decode('utf-8', errors="ignore").rstrip()
            # print("Serial read: " + str(line))     
            self.decode_angle(line)       
            time.sleep(0.01)
    
    def read_tof(self):
        for i in range(0,10,1):
            encoded_string = f"[222]".encode("utf-8")
            self.ser.write(encoded_string)
            line = self.ser.readline().decode('utf-8', errors="ignore").rstrip()
            # print("Serial read: " + str(line))
            self.decode_sensor(line)
            time.sleep(0.01)
        # print("serial write: " + str(self.encode_string()))
    
    def read_localization(self, init = False):
        for i in range(0,3,1):
            encoded_string = f"[322]".encode("utf-8")
            self.ser.write(encoded_string)
            line = self.ser.readline().decode('utf-8', errors="ignore").rstrip()
            print("Serial read: " + str(line))
            self.decode_localize(line)
            time.sleep(3)
        # print("serial write: " + str(self.encode_string()))

    def decode_angle(self,input_string):
        # Extracting numbers from the input_string using string manipulation
        # Line Format: "Wheels: [wl, wr]"
        if not input_string.startswith("Pico Angle"):
            return False
    
        try:
            start_idx = input_string.index("[") + 1
            end_idx = input_string.index("]")
            yaw_ang, lin_v = [float(num) for num in input_string[start_idx:end_idx].split(',')]
        except ValueError:
            print("Invalid input format")
            return False
        
        self.ang_data.update(yaw_ang, lin_v)
        # print(f"Updated Angle: {self.ang_data.yaw}")

        return True
    
    def decode_localize(self,input_string):
        # Extracting numbers from the input_string using string manipulation
        # Line Format: "Wheels: [wl, wr]"
        if not input_string.startswith("Pico Location"):
            return False
    
        try:
            start_idx = input_string.index("[") + 1
            end_idx = input_string.index("]")
            x, y = [float(num) for num in input_string[start_idx:end_idx].split(',')]
        except ValueError:
            print("Invalid input format")
            return False
        
        self.loc_data.update(x, y)
        # print(f"Updated Angle: {self.ang_data.yaw}")

        return True

        # Creating a SerialData object with the extracted numbers
    def decode_sensor(self,input_string):
        if not input_string.startswith("Pico Sensor"):
            return False
        
        try:
            start_idx = input_string.index("[") + 1
            end_idx = input_string.index("]")
            sensor_arr = [float(num) for num in input_string[start_idx:end_idx].split(',')]
        except ValueError:
            print("Invalid input format")
            return False
        
        self.sensor_data.update(sensor_arr)

        return True
       
class AngleData:
    """
    Data Structure for the Serial Orientation Data
    """
    def __init__(self, yaw=0, lin_v=0):
        self.update(yaw, lin_v)

    def update(self, yaw=None, lin_v=None):
        if yaw is not None:
            self.yaw = yaw
        if lin_v is not None:
            self.lin_v = lin_v
        
class LocalizationData:
    """
    Data Structure for the Serial Orientation Data
    """
    def __init__(self, x=0, y=0):
        self.update(x, y)

    def update(self, x=None, y=None):
        if x is not None:
            self.x = x / 1000
        if y is not None:
            self.y = y / 1000
        
class SensorData:
    """
    Data Structure for the Serial Orientation Data
    """
    def __init__(self, sensor_arr=None, sensor_len=8):
        # Initialize the sensor_arr attribute using np.empty or the provided sensor_arr.
        if sensor_arr is None:
            self.sensor_arr = np.empty(sensor_len, dtype=float)
        else:
            self.update(sensor_arr)

        # You don't need to update the array in the constructor.
        # The update method can be called separately.

    def update(self, sensor_arr):
        self.sensor_arr = sensor_arr
        

