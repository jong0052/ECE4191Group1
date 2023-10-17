# This file is the main code chunk running, and serves for multiprocessing.
from multiprocessing import *
import multiprocessing
from multiprocessing.managers import BaseManager, NamespaceProxy

from utils.mpManager import MPManager

from navigation_loop import *
from plotting_loop import *
from utils.const import *
from Communication.bluetoothManager import *
import os

if (not simulation):
    from serial_loop import *

class TestProxy(NamespaceProxy):
    # We need to expose the same __dunder__ methods as NamespaceProxy,
    # in addition to the b method.
    _exposed_ = ('__getattribute__', '__setattr__', '__delattr__', 'b')

    def b(self):
        callmethod = object.__getattribute__(self, '_callmethod')
        return callmethod('b')

def main():
    print(f"CPU Count: {multiprocessing.cpu_count()}")

    if (not simulation):
        gyro = Serializer_GT()
        gyro.activate()
        input("Waiting for Calibration. Press Enter for Ready.")

    # This is pretty epic workaround but it doesn't work with Objects in an Object. So for now,
    # supported is primitives and things supported by Managers.
    # Add Variables in utils/mpManager.py!
    BaseManager.register('MPManager', MPManager, TestProxy)
    manager = BaseManager()
    manager.start()
    manager_mp = manager.MPManager()
    
    proc1 = Process(target=state_test,args=(manager_mp,))

    if not simulation:
        proc2 = Process(target=serializer_loop, args=(manager_mp,))
        
    if bluetooth:
        if (host):
            procHost = Process(target=bluetooth_server,args=(manager_mp,))
        else:
            procClient = Process(target=bluetooth_client,args=(manager_mp,))
        # procUS = Process(target=usLoop, args=(manager_mp,))
    else:    
        procOtherRobot = Process(target=simulate_other_robot_loop,args=(manager_mp,))

    if plotting:
        proc3 = Process(target=plotting_loop, args=(manager_mp,))

    proc1.start()
    if not simulation:
        proc2.start()

    if bluetooth:
        if (host):
            procHost.start()
        else:
            procClient.start()
        # procUS.start()
    else:    
        procOtherRobot.start()

    if plotting:
        proc3.start()

    proc1.join()
    if not simulation:
        proc2.join()
        # procUS.join()
    if bluetooth:
        if (host):
            procHost.join()
        else:
            procClient.join()
    else:    
        procOtherRobot.join()   

    if plotting:
        proc3.join()

if __name__ == '__main__':
    main()
    

     


