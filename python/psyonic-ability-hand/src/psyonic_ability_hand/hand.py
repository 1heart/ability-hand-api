import asyncio
import array
import dataclasses
import struct
import time
import threading
from enum import Enum, IntEnum
from typing import Any, Protocol, List, Optional, Tuple, Union, Dict
from dataclasses import dataclass
from uuid import RESERVED_FUTURE
import numpy as np
import binascii
import queue
from .io import IOBase

from psyonic_ability_hand import log

# time in seconds to wait before triggering a query in case of packet drops
STALL_TIMEOUT = 0.250

HEX = binascii.hexlify



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

    def to_dict(self):
        return dataclasses.asdict(self)

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
    def unpack(status: int):
        return MotorHotStatus(
            bool(status & 0x1),
            bool(status & 0x2),
            bool(status & 0x4),
            bool(status & 0x8),
            bool(status & 0x10),
            bool(status & 0x20),
        )


@dataclass
class PressureData:
    NUM_PRESSURE_SITES = 6
    NUM_PRESSURE_FINGERS = 5
    Index: FingerPressure = FingerPressure()
    Middle: FingerPressure = FingerPressure()
    Ring: FingerPressure = FingerPressure()
    Pinky: FingerPressure = FingerPressure()
    ThumbFlexor: FingerPressure = FingerPressure()

    def to_dict(self):
        return dataclasses.asdict(self)


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


class MockComm(IOBase):
    POS_DATA = b"\x01\x02\x03" * 6
    TOUCH_DATA = b"\x95\x15\xC1" * 15
    STATUS = 0x3F

    def __init__(self):
        self._cond = threading.Condition()
        self._reply = b""
        self.pos_data = b"\0" * 12

    def read(self, n: int = 1) -> Optional[bytes]:
        with self._cond:
            while len(self._reply) < n:
                self._cond.wait()

            resp = self._reply[:n]
            self._reply = self._reply[n:]
            return resp

    def write(self, data: bytes) -> int:
        slave_address, command = data[:2]

        if command in (0x10, 0xA0):
            # position + current + touch
            if command == 0x10:
                self.pos_data = data[2 : 2 + 12]

            pos = self.pos_data
            cur = b'\0' * 12

            resp_data = b""

            for i in range(0, 24, 2):
                resp_data += pos[i : i + 2] + cur[i : i + 2]

            resp = (
                command.to_bytes(1, 'little')
                + resp_data
                + self.TOUCH_DATA
                + self.STATUS.to_bytes(1, 'little')
            )

            chk = checksum(resp)
            resp += chk.to_bytes(1, 'little')

            #time.sleep(.002)

            with self._cond:
                self._reply += resp
                self._cond.notify()

        return len(data)

    def __str__(self):
        return f"MockBus"


@dataclass
class HandStats:
    tx_bytes: int = 0
    rx_bytes: int = 0
    tx_packets: int = 0
    rx_packets: int = 0
    tx_bytes_per_sec: int = 0
    rx_bytes_per_sec: int = 0
    tx_packets_per_sec: float = 0
    rx_packets_per_sec: float = 0
    dt: float = 0.0
    rx_baud: int = 0
    tx_baud: int = 0


class Hand:
    def __init__(self, comm: IOBase, slave_address=0x50):
        log.debug(f"initializing hand over {comm}")
        self._comm = comm
        self._slave_address = slave_address
        self._run = True
        self._tx_thread = threading.Thread(target=self._tx)
        self._tx_packets = 0
        self._tx_bytes = 0
        self._tx_cond = threading.Condition()
        self._rx_thread = threading.Thread(target=self._rx)
        self._rx_packets = 0
        self._rx_bytes = 0
        self._pos_data = JointData()
        self._pos_update = 0
        self._pos_update_prev = 0
        self._torque_data = JointData()
        self._torque_update = 0
        self._torque_update_prev = 0
        self._velocity_data = JointData()
        self._velocity_update = 0
        self._velocity_update_prev = 0
        self._pwm_data = JointData()
        self._pwm_update = 0
        self._pwm_update_prev = 0

        self.width = 0
        self.max_width = 100.0
        self.is_grasped = False
        self.is_moving = False
        self._start_time = None
        self._stop_time = None
        self.position:JointData = JointData()
        self.current = JointData()
        self.velocity = JointData()
        self.touch = PressureData()
        self.motor_status = MotorHotStatus()

    def _tx(self):
        log.debug("tx thread starting")
        replyType = ReplyType.PositionCurrentTouch

        pos_updated = lambda: self._pos_update != self._pos_update_prev
        torque_updated = lambda: self._torque_update != self._torque_update_prev
        velocity_updated = lambda: self._velocity_update != self._velocity_update_prev
        pwm_updated = lambda: self._pwm_update != self._pwm_update_prev
        rx_ready = lambda: self._tx_packets == self._rx_packets

        while self._run:
            timed_out = False
            with self._tx_cond:
                while not rx_ready():
                    if timed_out := not self._tx_cond.wait(STALL_TIMEOUT):
                        break

            try:
                if pos_updated():
                    self.position_command(replyType, self._pos_data)
                    self._pos_update_prev = self._pos_update
                elif torque_updated():
                    self.torque_command(replyType, self._torque_data)
                    self._torque_update_prev = self._torque_update
                elif velocity_updated():
                    self.velocity_command(replyType, self._velocity_data)
                    self._velocity_update_prev = self._velocity_update
                elif pwm_updated():
                    self.pwm_command(replyType, self._pwm_data)
                    self._pwm_update_prev = self._pwm_update
                elif rx_ready() or timed_out:
                    if timed_out and self._run:
                        log.warning("tx forced restart due to stall")
                        self._tx_packets = self._rx_packets
                    self.query_command(replyType)
            except Exception as e:
                log.exception("tx error")
                self._run = False

        log.debug("tx thread ending")

    def _tx_notify(self):
        with self._tx_cond:
            self._tx_cond.notify()

    def _rx(self):
        log.debug("rx thread starting")
        while self._run:
            try:
                update = self.read()
            except ProtocolError:
                log.exception("protocol error")
                self._run = False
            except Exception as e:
                log.exception("unknown error")
                self._run = False

        log.debug("rx thread ending")

    def start(self):
        self._tx_packets = 0
        self._rx_packets = 0
        self._pos_update = (
            self._torque_update
        ) = self._velocity_update = self._pwm_update = 0
        self._pos_update_prev = (
            self._torque_update_prev
        ) = self._velocity_update_prev = self._pwm_update_prev = 0
        self._tx_thread.start()
        self._rx_thread.start()
        self._start_time = time.time()
        self._stop_time = None

    def stop(self):
        self._run = False
        self._rx_packets += 1
        self._tx_notify()
        self._tx_thread.join()
        self._tx_thread.join()
        self._stop_time = time.time()

    def stats(self):
        if self._start_time is None:
            return HandStats()

        stop_time = self._stop_time or time.time()

        dt = stop_time - self._start_time

        rx_bytes_per_sec = int(self._rx_bytes / dt)
        tx_bytes_per_sec = int(self._tx_bytes / dt)
        rx_baud = rx_bytes_per_sec * 10
        tx_baud = tx_bytes_per_sec * 10

        return HandStats(
            dt=dt,
            rx_bytes=self._rx_bytes,
            tx_bytes=self._tx_bytes,
            rx_packets=self._rx_packets,
            tx_packets=self._tx_packets or 0,
            rx_bytes_per_sec=rx_bytes_per_sec,
            tx_bytes_per_sec=tx_bytes_per_sec,
            rx_baud=rx_baud,
            tx_baud=tx_baud,
            tx_packets_per_sec=(self._tx_packets or 0) / dt,
            rx_packets_per_sec=self._rx_packets / dt,
        )

    def _command(self, command: int, data: bytes = b""):
        if self._slave_address:
            buf = struct.pack("BB", self._slave_address, command)
        else:
            buf = struct.pack("B", command)
        if data:
            buf += data

        buf += struct.pack("B", checksum(buf))
        #log.debug(f"sending command:{command:X}  data:{HEX(data)} : buf:{HEX(buf)}")
        self._tx_bytes += len(buf)
        self._tx_packets = (self._tx_packets or 0) + 1
        self._comm.write(buf)

    def upsample_thumb_rotator(self, enable: bool = True):
        self._command(0xC2 if enable else 0xC3)

    def exit_api_mode(self):
        self._command(0x7C)

    def set_grip(self, grip: Grip, speed: float = 0.10):
        """
        Set grip at speed 0-100% (0.0 - 1.0)
        """

        def scale(speed):
            speed = 0 if speed < 0 else 1.0 if speed > 1.0 else speed
            return int(speed * 100.0)

        log.debug(f"set grip: {grip} speed: {speed}")

        data = struct.pack("<BB", grip, scale(speed))
        self._command(0x1D, data)

    def query_command(self, replyType: ReplyType):
        self._command(0xA0 | replyType)

    def position_command(self, replyType: ReplyType, pos: JointData) -> None:
        """
        Joint position in degrees
        """

        def scale(p):
            LIMIT = 150
            p = -LIMIT if p < -LIMIT else LIMIT if p > LIMIT else p
            return int(p * 32767 / LIMIT) & 0xFFFF

        data = struct.pack("<6H", *[scale(p) for p in pos.to_list()])
        log.debug(f"position command: {pos} : {pos.to_list()} : {HEX(data)}")
        self._command(0x10 | replyType, data)

    def set_position(self, pos: JointData) -> None:
        self._pos_data = pos
        self._pos_update += 1
        self._tx_notify()

    def velocity_command(self, replyType: ReplyType, vel: JointData) -> None:
        """
        Joint velocities in degrees/sec
        """

        def scale(v):
            LIMIT = 3000
            v = -LIMIT if v < -LIMIT else LIMIT if v > LIMIT else v
            return int(v * 32767 / LIMIT)

        data = struct.pack("<6H", *[scale(v) for v in vel.to_list()])
        self._command(0x20 | replyType, data)

    def set_velocity(self, velocity: JointData):
        self._velocity_data = velocity
        self._velocity_update += 1
        self._tx_notify()

    def torque_command(self, replyType: ReplyType, torque: JointData) -> None:
        """
        Joint torques in mNM
        """

        def scale(t):
            LIMIT = 3.6
            t = -LIMIT if t < -LIMIT else LIMIT if t > LIMIT else t
            kt = 1.49  # nNM per Amp
            return int(t / kt * 7000 / 0.540)

        data = struct.pack("<6h", *[scale(t) for t in torque.to_list()])
        self._command(0x30 | replyType, data)

    def set_torque(self, torque: JointData) -> None:
        self._torque_data = torque
        self._torque_update += 1
        self._tx_notify()

    def pwm_command(self, replyType: ReplyType, pwm: JointData) -> None:
        """
        JointData should specify a PWM range or -100% to 100% for each joint
        """
        def scale(t):
            LIMIT = 3546
            t = -LIMIT if t < -LIMIT else LIMIT if t > LIMIT else t
            return int(t / 100 * LIMIT)

        data = struct.pack("<6h", *[scale(t) for t in pwm.to_list()])
        self._command(0x40 | replyType, data)

    def set_pwm(self, pwm: JointData) -> None:
        self._pwm_data = pwm
        self._pwm_update += 1
        self._tx_notify()

    def read(
        self,
    ) -> Optional[Dict[str, Union[JointData, PressureData, MotorHotStatus]]]:
        p_type = self._comm.read(1)

        # trigger TX thread as soon as first bytes detected
        with self._tx_cond:
            self._rx_bytes += 1
            self._rx_packets += 1
            self._tx_cond.notify()
            time.sleep(0)

        if p_type is None or len(p_type) != 1:
            raise ProtocolError("invalid/empty packet type")

        p_len = 0
        p_sum = 0

        format = p_type[0]

        variant = format & 0xF

        if variant == 2:
            p_len = 38
        else:
            p_len = 71

        p = self._comm.read(p_len)

        if p is None:
            return None

        self._rx_bytes += len(p)

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

        self.motor_status = MotorHotStatus.unpack(data[-1])

        # each finger: [position,current,..] or [position,velocity,...]
        d = struct.unpack("<12h", data[:24])

        decode_position = lambda pos: [p * 150 / 32767 for p in pos]
        decode_current = lambda qv: [d * 0.540 / 7000 for d in qv]
        decode_velocity = lambda qv: [d * 3000 / 32767 for d in qv]

        self.position = JointData( *decode_position(d[::2]) )
        qv = d[1::2]

        if variant == 0:
            self.current = JointData( *decode_current(qv) )
            self.touch = PressureData.unpack(data[24 : 24 + 45])
        elif variant == 1:
            self.velocity = JointData( *decode_velocity(qv) )
            self.touch = PressureData.unpack(data[24 : 24 + 45])
        elif variant == 2:
            # each finger: u16 rotor velocity
            self.current = JointData( *decode_current(qv) )
            self.velocity = JointData( *decode_velocity(struct.unpack("<6H", data[24:-1])) )
        else:
            raise ProtocolError(f"Unsupported reply variant; {variant}")


    def grasp(self, *, width, velocity=None, force=None):
        pass
