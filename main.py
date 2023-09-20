# This file is the main code chunk running, and serves for multiprocessing.
from multiprocessing import Process, Value, Manager
from multiprocessing.managers import BaseManager, NamespaceProxy

from utils.mpManager import MPManager

from navigation_loop import *

plotting = True
simulation = True

if (not simulation):
    from ultrasonic_loop import *
    from serial_loop import *

class TestProxy(NamespaceProxy):
    # We need to expose the same __dunder__ methods as NamespaceProxy,
    # in addition to the b method.
    _exposed_ = ('__getattribute__', '__setattr__', '__delattr__', 'b')

    def b(self):
        callmethod = object.__getattribute__(self, '_callmethod')
        return callmethod('b')

def main():
    # This is pretty epic workaround but it doesn't work with Objects in an Object. So for now,
    # supported is primitives and things supported by Managers.
    # Add Variables in utils/mpManager.py!
    BaseManager.register('MPManager', MPManager, TestProxy)
    manager = BaseManager()
    manager.start()
    manager_mp = manager.MPManager()
    
    proc1 = Process(target=navigation_loop,args=(manager_mp,))

    if not simulation:
        proc2 = Process(target=serializer_loop, args=(manager_mp,))
        procUS = Process(target=usLoop, args=(manager_mp,))

    if plotting:
        proc3 = Process(target=plotting_loop, args=(manager_mp,))

    proc1.start()
    if not simulation:
        proc2.start()
        procUS.start()
    if plotting:
        proc3.start()

    proc1.join()
    if not simulation:
        proc2.join()
        procUS.join()
    if plotting:
        proc3.join()

if __name__ == '__main__':
    main()
    

     


