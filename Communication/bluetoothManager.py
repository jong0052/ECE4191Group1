import uuid
import socket
import json
import time
import random
import math
from multiprocessing import *
from multiprocessing.managers import BaseManager, NamespaceProxy
from utils.mpManager import MPManager

port = 1

class BluetoothData():
    def __init__(self,
                 robot_state = 0, 
                 robot_pose = [0,0,0],
                 robot_goal = 0,
                 ):
        self.robot_state = robot_state # State of robot
        self.robot_pose = [0, 0, 0]
        for i in range(0, 2):
            self.robot_pose[i] = robot_pose[i] 
        self.robot_goal = robot_goal # Package ID (0 = A, 1 = B, 2 = C)

    def to_json(self):
        output = {
            "robot_state": self.robot_state,
            "robot_pose": self.robot_pose,
            "robot_goal": self.robot_goal
        }

        return json.dumps(output)

    def from_json(self, json_object):
        input = json.loads(json_object)

        self.robot_state = input["robot_state"]
        self.robot_pose = input["robot_pose"]
        self.robot_goal = input["robot_goal"]

    def to_string(self) -> str:
        return f"State: {self.robot_state}\nPose: {self.robot_pose}\nGoal: {self.robot_goal}"

def get_mac_address() -> str:
    return "D8:3A:DD:21:86:3C" # Pi B
    
    address = ':'.join(['{:02x}'.format((uuid.getnode() >> ele) & 0xff)
    for ele in range(0,8*6,8)][::-1])

    return address.upper()

# Client: Acts as the sender.
def bluetooth_client(mp_manager: MPManager):
    s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    host_address = "d8:3a:dd:21:87:5e" #Fill in host mac address here
    # address_other = "D8:3A:DD:21:80:55"
    s.connect((host_address,port))
    while(1):
        our_data = BluetoothData(mp_manager.robot_state, mp_manager.robot_data, mp_manager.robot_goal)
        our_json_data = our_data.to_json()

        s.send(bytes(our_json_data,'UTF-8'))

        other_data_json = s.recv(1024)
        print('I received: ', other_data_json)

        other_data = BluetoothData()
        other_data.from_json(other_data_json)
        print(other_data.to_string())

        mp_manager.other_robot_state = other_data.robot_state
        mp_manager.other_robot_pose[:] = []
        mp_manager.other_robot_pose.extend(other_data.robot_pose)
        mp_manager.other_robot_goal = other_data.robot_goal

        time.sleep(1)

    s.close()

# Server: Acts as the receiver.
def bluetooth_server(mp_manager: MPManager):
    s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    host_address = get_mac_address() #Fill in host mac address here
    print(f"Host Address: {host_address}")
    s.bind((host_address,port))

    s.listen(1)

    while True:
        try:
            client, address = s.accept()
            while 1:
                other_data_json = client.recv(1024)
                print('I received: ', other_data_json)

                other_data = BluetoothData()
                other_data.from_json(other_data_json)
                print(other_data.to_string())

                our_data = BluetoothData(mp_manager.robot_state, mp_manager.robot_data, mp_manager.robot_goal)
                our_json_data = our_data.to_json()
            
                client.send(bytes(our_json_data,'UTF-8'))
                
                mp_manager.other_robot_state = other_data.robot_state
                mp_manager.other_robot_pose[:] = []
                mp_manager.other_robot_pose.extend(other_data.robot_pose)
                mp_manager.other_robot_goal = other_data.robot_goal
        except:
            print("Closing socket")
            client.close()
            s.close()

        time.sleep(1)