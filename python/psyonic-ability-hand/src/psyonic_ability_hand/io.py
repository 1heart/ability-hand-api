import os
from typing import IO
from smbus2.smbus2 import SMBus, i2c_msg
from serial import Serial
from loguru import logger as log
from queue import Queue
from threading import Thread

class BusError(Exception):
    pass


class IOBase(IO):
    on_message = lambda *args: None
    send = lambda *args: None

    def __init__(self):
        self.q = Queue()


class I2CIO(IOBase):

    def __init__(self, bus_number=1, addr=0x50):
        self.bus = SMBus(bus_number, force=True)
        self.addr = addr
        self.txq = Queue()
        self.rxq = Queue()

        def do_io():
            pass

        self.thread = Thread(target=do_io)


    def read(self, len:int):
        msg = i2c_msg.read(self.addr, len)
        self.bus.i2c_rdwr(msg)
        return bytes(list(msg))

    def write(self, data:bytes):
        msg = i2c_msg.write(self.addr, data)
        self.bus.i2c_rdwr(msg)
        return len(data)

class SerialIO(IOBase):

    def __init__(self, device="/dev/ttyUSB0"):
        self.bus = Serial(device, 460800)

    def read(self, len:int):
        return self.bus.read(len)

    def write(self, data:bytes):
        return self.bus.write(data)
