import array
import dataclasses
import struct
import time
from enum import Enum, IntEnum
from typing import Any, IO, Protocol, List
from dataclasses import dataclass
import numpy as np

from loguru import logger as log

from .io import I2cIo


class Mode(IntEnum):
    Query = 0
    GripControl = 0x1D
    PosControl = 0xAD


class Grip(IntEnum):
    Open = 0
    PowerGrasp = 1
    KeyGrasp = 2
    PinchGrasp = 3
    ChuckGrasp = 4

    Point = 9

    Handshake = 17


@dataclass
class JointData:
    Index: float = 0
    Middle: float = 0
    Ring: float = 0
    Pinky: float = 0
    ThumbFlexor: float = 0
    ThumbRotate: float = 0

    def to_list(self):
        return dataclasses.astuple(self)


"""
Finger  | Index     | Middle    |  Ring     | Pinky     | ThumbF    | ThumbR
----
I: s0
M: s1




"""


@dataclass
class FingerPressure:
    s0: float = 0
    s1: float = 0
    s2: float = 0
    s3: float = 0
    s4: float = 0
    s5: float = 0


@dataclass
class PressureData:
    NUM_FINGERS = 5
    Index: FingerPressure = FingerPressure()
    Middle: FingerPressure = FingerPressure()
    Ring: FingerPressure = FingerPressure()
    Pinky: FingerPressure = FingerPressure()
    ThumbFlexor: FingerPressure = FingerPressure()

    @staticmethod
    def from_array(data: array.array):
        if len(data) != 30:
            raise HandException("not enough values for pressure data")

        pressures = []
        for finger in range(0, PressureData.NUM_FINGERS):
            i = finger * PressureData.NUM_FINGERS
            p = FingerPressure(*data[i : i + PressureData.NUM_FINGERS])
            log.info(f"i:{i}: {p}")
            pressures.append(p)

        return PressureData(*pressures)


class HandException(Exception):
    pass


class RcvError(Exception):
    pass


class Hand:
    def __init__(self, comm: IO):
        self._comm: IO = comm

    def checksum(self, d: array.array):
        v = np.cast["int8"](np.sum(d, dtype=np.int8))
        return -v

    def _txbuf(self) -> array.array:
        return array.array("B", [0] * 26)

    def write(self, data: array.array):
        data[-1] = self.checksum(data)
        ret = self._comm.write(data)
        if ret != len(data):
            raise HandException(f"write error: {ret}")

    def unpack(self, data) -> array.array:
        dout = array.array("H")
        if len(data) % 3 != 0:
            raise HandException(f"not enougn values ({len(data)}) to unpack")

        i = 0
        while i < len(data):
            end = i + 3
            (a, m, b) = data[i:end]
            i = end

            dout.append((a << 4) | (m >> 4))
            dout.append(((m & 0xF) << 8) | b)

        return dout

    def set_grip(self, grip: Grip, speed: float = 0.10):
        """
        Set grip at speed 0-100% (0.0 - 1.0)
        """
        cmd = Mode.GripControl

        if speed < 0 or speed > 1.0:
            raise HandException(f"speed {speed} out of range 0.0 - 1.0")

        speed_u8 = int(speed * 100.0)

        buf = struct.pack("<BBB", cmd, grip, speed_u8)

        ret = self._comm.write(buf)

        if ret != len(buf):
            raise HandException(f"incomplete write")

    def set_mode(self, mode: Mode):
        buf = self._txbuf()
        log.info(f"buf: {buf}")
        buf[0] = mode
        self.write(buf)

    def control_v1(self, mode: Mode, jointdata: JointData):
        """
        Writes JointData and returns current status
        """
        RXPACKET_SIZE = 71
        if mode != Mode.Query:
            buf = array.array("B", struct.pack("<B6fB", mode, *jointdata.to_list(), 0))
            self.write(buf)

        packet = self._comm.read(RXPACKET_SIZE)

        if len(packet) != RXPACKET_SIZE:
            raise RcvError()

        data = struct.unpack("<6f45BBB", packet)

        jd = JointData(*data[:6])
        pressure = PressureData.from_array(self.unpack(data[6:-2]))
        status = data[-2]
        checksum = data[-1]
        calculated_checksum = self.checksum(packet[:-1])

        if checksum != calculated_checksum:
            raise RcvError("checksum failed")

        return (jd, pressure, status)


if __name__ == "__main__":
    hand = Hand(I2cIo())

    hand.set_grip(Grip.Open)
    time.sleep(3.0)
    hand.set_grip(Grip.Handshake)
    time.sleep(3.0)
