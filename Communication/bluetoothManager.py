import uuid
import socket
import json

class BluetoothData():
    def __init__(self, 
                 pose = [0,0,0],
                 path= [[0,0,0]],
                 ):
        self.pose = pose  # x, y, th
        self.path = path  # Path Plan 

    def to_json(self):
        output = {
            "pose": self.pose,
            "path": self.path
        }

        return json.dumps(output)

    def from_json(self, json_object):
        input = json.loads(json_object)

        self.pose = input["pose"]
        self.path = input["path"]

class BluetoothManager():

    def __init__(self, port:int=1):
        self.socket = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        self.port = port
        self.address = self.get_mac_address()

    def get_mac_address(self) -> str:
        address = ':'.join(['{:02x}'.format((uuid.getnode() >> ele) & 0xff)
        for ele in range(0,8*6,8)][::-1])

        return address.upper()
    
    def open_server(self):
        self.socket.connect((self.address,self.port))
        while(1):
            text = input()
            if text == 'quit':
                break
            self.socket.send(bytes(text,'UTF-8'))
            data = self.socket.recv(1024)
            print ('I received', data)

        self.socket.close()
    
    def open_client(self):
        address_33 = "D8:3A:DD:21:80:55"
        self.socket.bind((self.address,self.port))

        self.socket.listen(1)
        try:
            client, address = self.socket.accept()
            while 1:
                data = client.recv(1024)
                if data:
                    print(data)
                client.send(bytes('Bananas','UTF-8'))
        except:
            print("Closing socket")
            client.close()
            self.socket.close()


if __name__ == "__main__":
    bM = BluetoothManager(2)
    print(bM.get_mac_address())

    bM.open_server()

    # data = BluetoothData([5,0.1,2], [[1, 2, 3], [3, 2, 1]])
    # output = data.to_json()

    # data2=  BluetoothData()
    # data2.from_json(output)
    # print(data2.path)
    # print(data2.pose)