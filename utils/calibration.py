import numpy as np
import os
import sys
import math
import time
import serial

sys.path.insert(0, "./../util")

class Serializer:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        self.ser.reset_input_buffer()
        self.data = SerialData()

    def read(self):
        line = self.ser.readline().decode('utf-8', errors="ignore").rstrip()
        print("serial read: " + str(line))
        self.decode_string(line)
    
    def write(self):
        self.ser.write(self.encode_string())
        print("serial write: " + str(self.encode_string()))

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


def calibrateWheelRadius():
    # Compute the robot scale parameter using a range of wheel velocities.
    # For each wheel velocity, the robot scale parameter can be computed
    # by comparing the time and distance driven to the input wheel velocities.

    ##########################################
    # Feel free to change the range / step
    ##########################################
    wheel_velocities_range = range(20, 30, 10)
    delta_times = []

    for wheel_vel in wheel_velocities_range:
        print("Driving at {} ticks/s.".format(wheel_vel))
        # Repeat the test until the correct time is found.
        while True:
            delta_time = input("Input the time to drive in seconds: ")
            try:
                delta_time = float(delta_time)
            except ValueError:
                print("Time must be a number.")
                continue

            # Drive the robot at the given speed for the given time
            drive_robot(1, 0, tick=wheel_vel, goal_time=delta_time)

            uInput = input("Did the robot travel 1m? [y/N]")
            if uInput == 'y':
                delta_times.append(delta_time)
                print("Recording that the robot drove 1m in {:.2f} seconds at wheel speed {}.\n".format(delta_time,
                                                                                                        wheel_vel))
                break

    # Once finished driving, compute the scale parameter by averaging
    num = len(wheel_velocities_range)
    scale = 0
    for delta_time, wheel_vel in zip(delta_times, wheel_velocities_range):
        scale += 1 / (delta_time * wheel_vel)
    scale = scale / num
    print("The scale parameter is estimated as {:.6f} m/ticks.".format(scale))

    return scale


def calibrateBaseline(scale):
    # Compute the robot basline parameter using a range of wheel velocities.
    # For each wheel velocity, the robot baseline parameter can be computed by
    # comparing the time elapsed and rotation completed to the input wheel
    # velocities to find out the distance between the wheels.

    ##########################################
    # Feel free to change the range / step
    ##########################################
    wheel_velocities_range = range(20, 30, 10)
    delta_times = []

    for wheel_vel in wheel_velocities_range:
        print("Driving at {} ticks/s.".format(wheel_vel))
        # Repeat the test until the correct time is found.
        while True:
            delta_time = input("Input the time to drive in seconds: ")
            try:
                delta_time = float(delta_time)
            except ValueError:
                print("Time must be a number.")
                continue

            # Spin the robot at the given speed for the given time
            drive_robot(0, 1, tick=20, turning_tick=wheel_vel, goal_time=delta_time)

            uInput = input("Did the robot spin 360deg?[y/N]")
            if uInput == 'y':
                delta_times.append(delta_time)
                print("Recording that the robot spun 360deg in {:.2f} seconds at wheel speed {}.\n".format(delta_time,
                                                                                                           wheel_vel))
                break

    # Once finished driving, compute the basline parameter by averaging
    num = len(wheel_velocities_range)
    baseline = 0
    for delta_time, wheel_vel in zip(delta_times, wheel_velocities_range):
        baseline += 2 * wheel_vel * delta_time 
        # pass # TODO: replace with your code to compute the baseline parameter using scale, wheel_vel, and delta_time

    baseline = scale * baseline / (2 * math.pi * num)
    print("The baseline parameter is estimated as {:.6f} m.".format(baseline))

    return baseline

def drive_robot(v, w, tick = 20, turning_tick=5, goal_time = 1):

    wl = v * tick - w * turning_tick
    wr = v * tick + w * turning_tick

    last = time.time()
    dt = 0

    while dt < goal_time:
        dt = time.time() - last

        serializer.data.update(wl_goal=wl, wr_goal=wr)
        serializer.write()
        time.sleep(0.2)
        serializer.read()

    
    serializer.data.update(wl_goal=0, wr_goal=0)
    serializer.write()
    time.sleep(0.2)

    serializer.read()
    
serializer = Serializer()

if __name__ == '__main__':
    serializer.data.update(wl_goal=0, wr_goal=0)
    serializer.write()

    dataDir = "{}/param/".format(os.getcwd())

    print('Calibrating PiBot scale...\n')
    scale = calibrateWheelRadius()
    #fileNameS = "{}scale.txt".format(dataDir)
    #np.savetxt(fileNameS, np.array([scale]), delimiter=',')
    
    scale = 0.03

    print('Calibrating PiBot baseline...\n')
    baseline = calibrateBaseline(scale)
    fileNameB = "{}baseline.txt".format(dataDir)
    np.savetxt(fileNameB, np.array([baseline]), delimiter=',')

    print('Finished calibration')    