import os
from typing import Optional
from smbus2.smbus2 import SMBus, i2c_msg
from serial import Serial
from queue import Queue
from threading import Thread


class IOBase:
    def read(self, len: int) -> Optional[bytes]:
        pass

    def write(self, data: bytes) -> int:
        return 0

class I2CIO(IOBase):
    def __init__(self, bus_number=1, addr=0x50):
        self.bus = SMBus(bus_number, force=True)
        self.addr = addr

    def read(self, len: int):

        msg = i2c_msg.read(self.addr, len)
        self.bus.i2c_rdwr(msg)
        return bytes(list(msg))

    def write(self, data: bytes):
        msg = i2c_msg.write(self.addr, data)
        self.bus.i2c_rdwr(msg)
        return len(data)

    def __str__(self):
        return f'I2C[{self.bus}]@0x{self.addr:x}'

class SerialIO(IOBase):
    def __init__(self, device="/dev/ttyUSB0"):
        self.bus = Serial(device, 460800)

    def read(self, len: int) -> Optional[bytes]:
        return self.bus.read(len)

    def write(self, data: bytes) -> int:
        return self.bus.write(data)

    def __str__(self):
        return f'{self.bus}'