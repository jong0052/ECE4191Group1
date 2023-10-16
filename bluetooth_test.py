from Communication.bluetoothManager import *


class TestProxy(NamespaceProxy):
    # We need to expose the same __dunder__ methods as NamespaceProxy,
    # in addition to the b method.
    _exposed_ = ('__getattribute__', '__setattr__', '__delattr__', 'b')

    def b(self):
        callmethod = object.__getattribute__(self, '_callmethod')
        return callmethod('b')


if __name__ == "__main__":
    BaseManager.register('MPManager', MPManager, TestProxy)
    manager = BaseManager()
    manager.start()
    manager_mp = manager.MPManager()
    bluetooth_server(manager_mp)