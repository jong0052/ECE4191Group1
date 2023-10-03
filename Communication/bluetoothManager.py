import uuid
import socket
import json


class BluetoothManager():

    def __init__(self, port:int=1):
        pass

    def get_mac_address(self) -> str:
        address = ':'.join(['{:02x}'.format((uuid.getnode() >> ele) & 0xff)
        for ele in range(0,8*6,8)][::-1])

        print(address)

        return address


if __name__ == "__main__":
    bM = BluetoothManager()
    bM.get_mac_address()