import uuid
import socket
import json
import time
import random
import math
from multiprocessing import *
from multiprocessing.managers import BaseManager, NamespaceProxy

port = 55

class BluetoothMPManager:
    def __init__(self):
        manager = Manager()

        self.our_robot_state = 0
        self.our_robot_pose = manager.list()
        self.our_robot_goal = 0

        self.other_robot_state = 0
        self.other_robot_pose = manager.list()
        self.other_robot_goal = 0

    def to_string(self) -> str:
        return f"Our State: {self.our_robot_state}\nOur Pose: {self.our_robot_pose}\nOur Goal: {self.our_robot_goal}\nOther State: {self.other_robot_state}\nOther Pose: {self.other_robot_pose}\nOther Goal: {self.other_robot_goal}" 

class TestProxy(NamespaceProxy):
    # We need to expose the same __dunder__ methods as NamespaceProxy,
    # in addition to the b method.
    _exposed_ = ('__getattribute__', '__setattr__', '__delattr__', 'b')

    def b(self):
        callmethod = object.__getattribute__(self, '_callmethod')
        return callmethod('b')

class BluetoothData():
    def __init__(self,
                 robot_state = 0, 
                 robot_pose = [0,0,0],
                 robot_goal = 0,
                 ):
        self.robot_state = robot_state # State of robot
        self.robot_pose = robot_pose  # x, y, th
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
    address = ':'.join(['{:02x}'.format((uuid.getnode() >> ele) & 0xff)
    for ele in range(0,8*6,8)][::-1])

    return address.upper()

# Client: Acts as the sender.
def bluetooth_client(bluetooth_manager: BluetoothMPManager):
    s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    host_address = "d8:3a:dd:21:86:59" #Fill in host mac address here
    # address_other = "D8:3A:DD:21:80:55"
    s.connect((host_address,port))
    while(1):
        our_data = BluetoothData(bluetooth_manager.our_robot_state, bluetooth_manager.our_robot_pose, bluetooth_manager.our_robot_goal)
        our_json_data = our_data.to_json()

        s.send(bytes(our_json_data,'UTF-8'))

        other_data_json = s.recv(1024)
        print('I received: ', other_data_json)

        other_data = BluetoothData()
        other_data.from_json(other_data_json)
        print(other_data.to_string())

        bluetooth_manager.other_robot_state = other_data.robot_state
        bluetooth_manager.other_robot_pose[:] = []
        bluetooth_manager.other_robot_pose.extend(other_data.robot_state)
        bluetooth_manager.other_robot_goal = other_data.robot_state

        time.sleep(1)

    s.close()

# Server: Acts as the receiver.
def bluetooth_server(bluetooth_manager: BluetoothMPManager):
    s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    host_address = get_mac_address() #Fill in host mac address here
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

                our_data = BluetoothData(bluetooth_manager.our_robot_state, bluetooth_manager.our_robot_pose, bluetooth_manager.our_robot_goal)
                our_json_data = our_data.to_json()
            
                client.send(bytes(our_json_data,'UTF-8'))
                
                bluetooth_manager.other_robot_state = other_data.robot_state
                bluetooth_manager.other_robot_pose[:] = []
                bluetooth_manager.other_robot_pose.extend(other_data.robot_state)
                bluetooth_manager.other_robot_goal = other_data.robot_state
        except:
            print("Closing socket")
            client.close()
            s.close()

        time.sleep(1)

def random_update(bluetooth_manager: BluetoothMPManager):
    while True:
        bluetooth_manager.our_robot_state = random.randint(0, 5)
        bluetooth_manager.our_robot_pose[:] = []
        bluetooth_manager.our_robot_pose.extend([random.random() * 2, random.random() * 2, (random.random() - 0.5) * math.pi * 2])
        bluetooth_manager.our_robot_goal = random.randint(0, 5)

        print(bluetooth_manager.to_string())
        
        time.sleep(5)

if __name__ == "__main__":
    BaseManager.register('BluetoothMPManager', BluetoothMPManager, TestProxy)
    manager = BaseManager()
    manager.start()
    bluetooth_manager = manager.BluetoothMPManager()

    host = False

    if (host):
        procHost = Process(target=bluetooth_server,args=(bluetooth_manager,))
        procHost.start()
        procHost.join()

    else:
        procClient = Process(target=bluetooth_client,args=(bluetooth_manager,))
        procClient.start()
        procClient.join()

    procRandom = Process(target=random_update, args=(bluetooth_manager,))
    procRandom.start()
    procRandom.join()
    