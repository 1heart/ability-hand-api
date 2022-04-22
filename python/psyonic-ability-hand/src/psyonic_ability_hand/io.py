from typing import IO
from smbus2.smbus2 import SMBus, i2c_msg

class I2cIo(IO):

    def __init__(self, bus_number=1, addr=0x50):
        self.bus = SMBus(bus_number)
        self.addr = addr

    def read(self, len:int):
        msg = i2c_msg.read(self.addr, len)
        self.bus.i2c_rdwr(msg)
        return msg.buf

    def write(self, data:bytes):
        msg = i2c_msg.write(self.addr, data)
        self.bus.i2c_rdwr(msg)
        return len(data)