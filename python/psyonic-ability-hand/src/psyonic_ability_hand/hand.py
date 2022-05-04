import asyncio
import array
import dataclasses
import struct
import time
from enum import Enum, IntEnum
from typing import Any, IO, Protocol, List, Optional, Tuple, Union, Dict
from dataclasses import dataclass
from uuid import RESERVED_FUTURE
import numpy as np
import binascii
import queue


HEX = binascii.hexlify

from loguru import logger as log

class ProtocolError(Exception):
    pass

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
    ThumbRotator: float = 0

    def to_list(self):
        return dataclasses.astuple(self)

@dataclass
class FingerPressure:
    s0: float = 0
    s1: float = 0
    s2: float = 0
    s3: float = 0
    s4: float = 0
    s5: float = 0

@dataclass
class MotorHotStatus:
    Index: bool = False
    Middle: bool = False
    Ring: bool = False
    Pinky: bool = False
    ThumbFlexor: bool = False
    ThumbRotator: bool = False

    @staticmethod
    def unpack(status:int) :
        return MotorHotStatus(
            bool( status & 0x1 ),
            bool(status & 0x2),
            bool(status & 0x4),
            bool(status & 0x8),
            bool(status & 0x10),
            bool(status & 0x20) )

@dataclass
class PressureData:
    NUM_PRESSURE_SITES = 6
    NUM_PRESSURE_FINGERS = 5
    Index: FingerPressure = FingerPressure()
    Middle: FingerPressure = FingerPressure()
    Ring: FingerPressure = FingerPressure()
    Pinky: FingerPressure = FingerPressure()
    ThumbFlexor: FingerPressure = FingerPressure()

    @staticmethod
    def unpack(data):
        if len(data) != 45:
            raise ProtocolError(f"unexpected data length: {len(data)}")

        def decode_triplets(data):
            for i in range(0, len(data) - 3, 3):
                (a, m, b) = data[i : i + 3]
                yield float((a << 4) | (m >> 4))
                yield float(((m & 0xF) << 8) | b)

        # convert 45-byte packed nibbles to list of 30 (6 sensors, 5 fingers with sensors) flaots
        data = [d for d in decode_triplets(data)]

        # chunk per finger
        return PressureData(
            *[
                FingerPressure(*data[i : i + PressureData.NUM_PRESSURE_SITES])
                for i in range(0, len(data), PressureData.NUM_PRESSURE_SITES)
            ]
        )


class ReplyType(IntEnum):
    PositionCurrentTouch = 0
    PositionVelocityTouch = 1
    PositionCurrentVelocity = 2

GEAR_RATIO = 649
GEAR_RATIO_TR = 162.45

def checksum(data):
    cksum = 0
    for d in data:
        cksum = cksum + d
    return (-cksum) & 0xFF


class MockHandComm:
    POS_DATA = b'\x01\x02\x03' * 6
    TOUCH_DATA = b'\x95\x15\xC1' * 15
    STATUS = 0x3f

    def __init__(self):
        self.reply_q = queue.Queue()
        self.pos_data = b'\0' * 12

    def read(self, n:int=1) -> bytes:
        buf = []
        for i in range(n):
            buf.append( self.reply_q.get() )
        return bytes( buf )

    def write(self, data:bytes) -> int:
        slave_address,command = struct.unpack('BB', data[:2])

        if command in (0x10, 0xA0):
            # position + current + touch
            if command == 0x10:
                self.pos_data = data[2:2+12]

            pos = self.pos_data
            cur = bytes([0] * 12)

            resp_data = b''

            for i in range(0, 24, 2):
                resp_data += pos[i:i+2] + cur[i:i+2]

            resp = struct.pack('B', command) + resp_data + self.TOUCH_DATA + struct.pack('B', self.STATUS)
            resp += struct.pack('B', checksum(resp))
            for b in resp:
                self.reply_q.put_nowait(b)

        return len(data)

class Hand:
    def __init__(self, comm: IO, slave_address=0x50):
        log.debug(f"initializing hand with comm {comm}")
        self._comm = comm
        self._slave_address = slave_address

    def _command(self, command:int, data:bytes=b''):
        if self._slave_address:
            buf = struct.pack("BB", self._slave_address, command)
        else:
            buf = struct.pack('B', command)
        if data:
            buf += data

        buf += struct.pack("B", checksum(buf))
        log.debug(f"sending command:{command:X}  data:{HEX(data)} : buf:{HEX(buf)}")
        self._comm.write(buf)

    def upsample_thumb_rotator(self, enable:bool=True):
        self._command( 0xC2 if enable else 0xC3 )

    def exit_api_mode(self):
        self._command( 0x7C )

    def set_grip(self, grip: Grip, speed: float = 0.10):
        """
        Set grip at speed 0-100% (0.0 - 1.0)
        """
        def scale(speed):
            speed = 0 if speed < 0 else 1.0 if speed > 1.0 else speed
            return int(speed * 100.0)

        log.debug(f"set grip: {grip} speed: {speed}")

        data = struct.pack("<BB", grip, scale(speed))
        self._command( 0x1D, data )

    def query_command(self, replyType:ReplyType):
        self._command(0xA0 | replyType)

    def position_command(self, replyType:ReplyType, pos: JointData) -> None:
        """
        Joint position in degrees
        """
        def scale(p):
            LIMIT=150
            p = -LIMIT if p < -LIMIT else LIMIT if p > LIMIT else p
            return int(p * 32767 / LIMIT) & 0xFFFF
        data = struct.pack('<6H', *[ scale(p)  for p in pos.to_list() ])
        log.debug(f"position command: {pos} : {pos.to_list()} : {HEX(data)}")
        self._command(0x10 | replyType, data)

    def velocity_command(self, replyType:ReplyType, vel: JointData) -> None:
        """
        Joint velocities in degrees/sec
        """
        def scale(v):
            LIMIT=3000
            v = -LIMIT if v < -LIMIT else LIMIT if v > LIMIT else v
            return int(v * 32767/LIMIT)
        data = struct.pack('<6H', *[ scale(v) for v in vel.to_list()  ])
        self._command(0x20 | replyType, data)

    def torque_command(self, replyType:ReplyType, torque: JointData) -> None:
        """
        Joint torques in mNM
        """
        def scale(t):
            kt = 1.49 # nNM per Amp
            return int(t / kt * 7000 / 0.540) & 0xFFFF
        data = struct.pack('<6h', *[ scale(t) for t in torque.to_list() ])
        self._command(0x30 | replyType, data)

    def pwm_command(self, replyType:ReplyType, pwm: JointData) -> None:
        """
        JointData should specify a PWM range or -100% to 100% for each joint
        """
        def scale(t):
            LIMIT = 3546
            t = -LIMIT if t < -LIMIT else LIMIT if t > LIMIT else t
            return int(t / 100 * LIMIT) & 0xFFFF

        data = struct.pack('<6h', *[ scale(t) for t in pwm.to_list() ])
        self._command(0x40 | replyType, data)

    def read(self) -> Dict[str, Union[JointData,PressureData,MotorHotStatus]]:
        p_type = self._comm.read(1)

        if p_type is None or len(p_type) != 1:
            raise ProtocolError("invalid/empty packet type")

        p_len = 0
        p_sum = 0

        format = struct.unpack( 'B', p_type )[0]

        variant = format & 0xF

        if variant == 2:
            p_len = 38
        else:
            p_len = 71

        p = self._comm.read(p_len)

        log.debug(f"received: {HEX(p)}")

        if p is None or len(p) != p_len:
            raise ProtocolError("invalid length")

        p_sum = p[-1]
        # strip received checksum for checksum calc
        data = p[:-1]
        # reassemble header for checksum calc
        p = p_type + data

        if p_sum != checksum(p):
            raise ProtocolError(
                f"checksum failed: received {p_sum} expected: {checksum(p)}"
            )

        # variant 0: [ (position, motor current), ... ]  | [ touch sensor, ... ] | hot/cold status
        # variant 1: [ (position, rotor velocity), ... ] | [ touch sensor, ... ] | hot/cold status
        # variant 2: [ (position, motor current), ... ] | [ rotor velocity, ... ] | hot/cold status

        # voltage: -3546 -3546 = -100% - 100% PWM
        pos = []
        cur = []
        vel = []
        status = MotorHotStatus()
        touch = PressureData()

        # each finger: [(position,current),] or [(position,velocity),]
        d = struct.unpack("<12h", data[:24])
        pos, qv = zip(*[(d[i], d[i + 1]) for i in range(0, len(d), 2)])

        decode_position = lambda pos: [ p * 150 / 32767 for p in pos ]
        decode_current = lambda qv: [ d * .540/7000 for d in qv ]
        decode_velocity = lambda qv: [  d * 3000/32767 for d in qv  ]

        pos = decode_position(pos)

        if variant == 0:
            cur = decode_current(qv)
            touch = PressureData.unpack( data[24 : 24 + 45] )
        elif variant == 1:
            vel = decode_velocity(qv)
            touch = PressureData.unpack( data[24 : 24 + 45] )
        elif variant == 2:
            # each finger: u16 rotor velocity
            cur = decode_current(qv)
            vel = decode_velocity( struct.unpack("<6H", data[24:-1]) )
        else:
            raise ProtocolError(f"Unsupported reply variant; {variant}")

        return {
            'position' : JointData(*pos),
            'current' : JointData(*cur),
            'velocity' : JointData(*vel),
            'touch' : touch,
            'status' : MotorHotStatus.unpack( data[-1] )
            }
