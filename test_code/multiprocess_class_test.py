from multiprocessing import Process, Value, Manager
from multiprocessing.managers import BaseManager, NamespaceProxy
import time

class MyClass():
    def __init__(self):
        self.number = 0

def add_loop(object):
    while (True):
        time.sleep(1)
        object.number = object.number + 1

def print_loop(object):
    while (True):
        time.sleep(0.1)
        print(object.number)

class TestProxy(NamespaceProxy):
    # We need to expose the same __dunder__ methods as NamespaceProxy,
    # in addition to the b method.
    _exposed_ = ('__getattribute__', '__setattr__', '__delattr__', 'b')

    def b(self):
        callmethod = object.__getattribute__(self, '_callmethod')
        return callmethod('b')
    
if __name__ == "__main__":
    BaseManager.register('MyClass', MyClass, TestProxy)
    manager = BaseManager()
    manager.start()
    inst = manager.MyClass()

    proc1 = Process(target=add_loop,args=(inst,))
    proc2 = Process(target=print_loop,args=(inst,))

    proc1.start()
    proc2.start()
    proc1.join()
    proc2.join()
