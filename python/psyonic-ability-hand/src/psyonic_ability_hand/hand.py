import asyncio
import array
import dataclasses
import struct
import time
import threading
from enum import Enum, IntEnum
from typing import List, Optional, Union, Dict, Any, Tuple, Callable
from dataclasses import dataclass
from uuid import RESERVED_FUTURE
import numpy as np
import binascii
import queue
from .io import IOBase

from psyonic_ability_hand import log

# time in seconds to wait before triggering a query in case of packet drops
STALL_TIMEOUT = 0.250

POSITION_LIMIT_MIN = 0.0
POSITION_LIMIT_MAX = 150.0

VELOCITY_LIMIT_MIN = 0
VELOCITY_LIMIT_MAX = 100.0

TORQUE_LIMIT_MIN = -3.2
TORQUE_LIMIT_MAX = 3.2

PWM_LIMIT_MIN = 0
PWM_LIMIT_MAX = 100.0

V1_READ_ONLY = 0x31
V1_POS_UPDATE = 0xAD
V1_VELOCITY_UPDATE = 0xAC
V1_TORQUE_UPDATE = 0xAB

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


NUM_TOUCH_SITES = 6
NUM_TOUCH_FINGERS = 5
EMPTY_TOUCH = (0.0,) * NUM_TOUCH_SITES


@dataclass
class TouchSensors:
    Index: Tuple[float] = EMPTY_TOUCH
    Middle: Tuple[float] = EMPTY_TOUCH
    Ring: Tuple[float] = EMPTY_TOUCH
    Pinky: Tuple[float] = EMPTY_TOUCH
    ThumbFlexor: Tuple[float] = EMPTY_TOUCH

    def to_dict(self):
        return dataclasses.asdict(self)

    @staticmethod
    def unpack(data):
        if len(data) != 45:
            raise ProtocolError(f"unexpected data length: {len(data)}")

        def decode_triplets(data):
            for i in range(0, len(data), 3):
                (a, m, b) = data[i : i + 3]
                yield float((a << 4) | (m >> 4))
                yield float(((m & 0xF) << 8) | b)

        # convert 45-byte packed nibbles to list of 30 (6 sensors, 5 fingers with sensors) flaots
        data = [d for d in decode_triplets(data)]

        # chunk per finger
        return TouchSensors(
            *(
                tuple(data[i : i + NUM_TOUCH_SITES])
                for i in range(0, len(data), NUM_TOUCH_SITES)
            )
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
    TOUCH_DATA = b"\x00\x10\x02\x00\x30\x04\x00\x50\x06" * 5
    STATUS = 0x3F

    def __init__(self):
        self._cond = threading.Condition()
        self._reply = b""
        self.pos_data = b"\0" * 12

    def read(self, n: int = 1) -> Optional[bytes]:
        with self._cond:
            while len(self._reply) < n:
                timed_out = not self._cond.wait(timeout=1.0)
                if timed_out:
                    raise Exception("read timeout")

            resp = self._reply[:n]
            self._reply = self._reply[n:]
            return resp

    def write(self, data: bytes) -> int:
        slave_address, command = data[:2]

        if command in (0x1D,):
            pass
        elif command in (0x10, 0xA0):
            # position + current + touch
            if command == 0x10:
                self.pos_data = data[2 : 2 + 12]

            pos = self.pos_data
            cur = b"\0" * 12

            resp_data = b""

            for i in range(0, 24, 2):
                resp_data += pos[i : i + 2] + cur[i : i + 2]

            resp = (
                command.to_bytes(1, "little")
                + resp_data
                + self.TOUCH_DATA
                + self.STATUS.to_bytes(1, "little")
            )

            chk = checksum(resp)
            resp += chk.to_bytes(1, "little")

            time.sleep(.002)

            with self._cond:
                self._reply += resp
                self._cond.notify()

        else:
            raise Exception(f"unsupported link command: 0x{command:x}")

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


class HandThread(threading.Thread):
    def __init__(self, hand: "Hand"):
        super().__init__()
        self.exception = None
        self.hand = hand


class HandTxThread(HandThread):
    def run(self):
        hand = self.hand
        log.debug("tx thread starting")
        replyType = ReplyType.PositionCurrentTouch

        pos_updated = lambda: hand._pos_update != hand._pos_update_prev
        torque_updated = lambda: hand._torque_update != hand._torque_update_prev
        velocity_updated = lambda: hand._velocity_update != hand._velocity_update_prev
        pwm_updated = lambda: hand._pwm_update != hand._pwm_update_prev
        rx_ready = lambda: hand._tx_packets == hand._rx_packets

        while hand._run:
            timed_out = False
            with hand._tx_cond:
                while not rx_ready():
                    timed_out = not hand._tx_cond.wait(STALL_TIMEOUT)
                    if timed_out:
                        hand._on_error(f"Tx/Rx sync error")
                        break

            if timed_out:
                hand._tx_packets = hand._rx_packets
                continue

            try:
                if pos_updated():
                    hand.position_command(replyType, hand._pos_data)
                    hand._pos_update_prev = hand._pos_update
                elif torque_updated():
                    hand.torque_command(replyType, hand._torque_data)
                    hand._torque_update_prev = hand._torque_update
                elif velocity_updated():
                    hand.velocity_command(replyType, hand._velocity_data)
                    hand._velocity_update_prev = hand._velocity_update
                elif pwm_updated():
                    hand.pwm_command(replyType, hand._pwm_data)
                    hand._pwm_update_prev = hand._pwm_update
                elif rx_ready():
                    hand.query_command(replyType)
            except Exception as e:
                hand._on_error(f"TX error: {e}")
                hand._pos_update_prev = hand._pos_update
                hand._torque_update_prev = hand._torque_update
                hand._velocity_update_prev = hand._velocity_update
                hand._pwm_update_prev = hand._pwm_update

        log.debug("tx thread ending")


class HandRxThread(HandThread):
    def run(self):
        hand = self.hand
        log.debug("rx thread starting")
        while hand._run:
            try:
                if hand.is_v1():
                    hand.read_v1()
                   # V1/I2C will always immediately return with data
                    time.sleep(0.030)
                else:
                    hand.read_v2()
            except Exception as e:
                hand._on_error(f"RX error: {e}")
                hand._run = False

        log.debug("rx thread ending")


class Hand:
    def __init__(
        self,
        comm: IOBase,
        slave_address=0x50,
        protocol_version=1,
        on_error: Optional[Callable[[str], None]] = None,
    ):
        self._comm = comm
        self._slave_address = slave_address
        self._on_error = on_error or (lambda msg: log.error(msg))
        self._run = True
        self._tx_thread = HandTxThread(self)
        self._tx_packets = 0
        self._tx_bytes = 0
        self._command_prev = 0 #for v1/i2c, track the last command sent to know what size packet to read
        self._tx_cond = threading.Condition()
        self._rx_thread = HandRxThread(self)
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
        self._protocol_version = protocol_version

        self.width = 0
        self.max_width = 100.0
        self.is_grasped = False
        self.is_moving = False
        self._start_time = None
        self._stop_time = None
        self._position: JointData = JointData()
        self._current = JointData()
        self._velocity = JointData()
        self._touch = TouchSensors()
        self._motor_status = MotorHotStatus()

        log.debug(
            f"initializing hand over {comm} protocol version: { self._protocol_version } "
        )

    def is_v1(self):
        return self._protocol_version == 1

    def is_v2(self):
        return self._protocol_version == 2

    def get_position(self):
        return self._position

    def set_position(self, pos: JointData) -> None:
        self._pos_data = pos
        log.debug(f"position update: {pos}")
        self._pos_update += 1
        self._tx_notify()

    def get_current(self):
        return self._current

    def set_torque(self, torque: JointData) -> None:
        self._torque_data = torque
        log.debug(f"torque update: {torque}")
        self._torque_update += 1
        self._tx_notify()

    def get_velocity(self):
        return self._velocity

    def set_velocity(self, velocity: JointData):
        self._velocity_data = velocity
        log.debug(f"velocity update: {velocity}")
        self._velocity_update += 1
        self._tx_notify()

    def set_pwm(self, pwm: JointData) -> None:
        self._pwm_data = pwm
        log.debug(f"pwm update: {pwm}")
        self._pwm_update += 1
        self._tx_notify()

    def get_touch(self):
        return self._touch

    def get_motor_status(self):
        return self._motor_status

    position = property(get_position)
    current = property(get_current)
    velocity = property(get_velocity)
    touch = property(get_touch)
    motor_status = property(get_motor_status)

    def _tx_notify(self):
        with self._tx_cond:
            self._tx_cond.notify()

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
        stats = HandStats(
            rx_bytes=self._rx_bytes,
            tx_bytes=self._tx_bytes,
            rx_packets=self._rx_packets,
            tx_packets=self._tx_packets or 0,
        )

        if not self._run or not self._start_time:
            return stats

        stop_time = self._stop_time or time.time()

        stats.dt = stop_time - self._start_time

        stats.rx_bytes_per_sec = int(self._rx_bytes / stats.dt)
        stats.tx_bytes_per_sec = int(self._tx_bytes / stats.dt)
        stats.rx_baud = stats.rx_bytes_per_sec * 10
        stats.tx_baud = stats.tx_bytes_per_sec * 10
        stats.tx_packets_per_sec = (self._tx_packets or 0) / stats.dt
        stats.rx_packets_per_sec = self._rx_packets / stats.dt

        return stats

    def _command(self, command: int, data: bytes = b""):

        if self.is_v1() and command == V1_READ_ONLY:
            #i2c mode, nothing sent to just get current status
            return

        if self.is_v2() and self._slave_address is not None:
            buf = struct.pack("BB", self._slave_address, command)
        else:
            buf = command.to_bytes(1, "little")

        if data:
            buf += data

        self._command_prev = command

        buf += struct.pack("B", checksum(buf))
        self._comm.write(buf)

        self._tx_bytes += len(buf)
        self._tx_packets = (self._tx_packets or 0) + 1

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
        cmd = (0xA0 | replyType) if self.is_v2() else V1_READ_ONLY
        self._command(cmd)

    def position_command(self, replyType: ReplyType, pos: JointData) -> None:
        """
        Joint position in degrees
        """
        def set_mode(mode):
            d = struct.pack('25B', mode, *([0]*24) )
            d += checksum(d).to_bytes(1, 'little')
            self._comm.write(d)

        if self.is_v1():
            set_mode(V1_POS_UPDATE)
            self._command( V1_POS_UPDATE, struct.pack('<6f', *pos.to_list() ) )
            return

        def scale(p):
            LIMIT = 150
            p = -LIMIT if p < -LIMIT else LIMIT if p > LIMIT else p
            return int(p * 32767 / LIMIT) & 0xFFFF

        data = struct.pack("<6H", *[scale(p) for p in pos.to_list()])
        self._command(0x10 | replyType, data)

    def velocity_command(self, replyType: ReplyType, vel: JointData) -> None:
        """
        Joint velocities in degrees/sec
        """

        if self.is_v1():
            return

        def scale(v):
            LIMIT = 3000
            v = -LIMIT if v < -LIMIT else LIMIT if v > LIMIT else v
            return int(v * 32767 / LIMIT)

        data = struct.pack("<6H", *[scale(v) for v in vel.to_list()])
        self._command(0x20 | replyType, data)

    def torque_command(self, replyType: ReplyType, torque: JointData) -> None:
        """
        Joint torques in mNM
        """

        if self.is_v1():
            return

        def scale(t):
            LIMIT = 3.6
            t = -LIMIT if t < -LIMIT else LIMIT if t > LIMIT else t
            kt = 1.49  # nNM per Amp
            return int(t / kt * 7000 / 0.540)

        data = struct.pack("<6h", *[scale(t) for t in torque.to_list()])
        self._command(0x30 | replyType, data)

    def pwm_command(self, replyType: ReplyType, pwm: JointData) -> None:
        """
        JointData should specify a PWM range or -100% to 100% for each joint
        """

        if self.is_v1():
            return

        def scale(t):
            LIMIT = 3546
            t = -LIMIT if t < -LIMIT else LIMIT if t > LIMIT else t
            return int(t / 100 * LIMIT)

        data = struct.pack("<6h", *[scale(t) for t in pwm.to_list()])
        self._command(0x40 | replyType, data)

    def read_v1(self):
        p_len = 71
        p = self._comm.read(p_len)
        log.debug(f"RX@{len(p)}: {HEX(p)}")
        if p is None or len(p) != p_len:
            raise ProtocolError("invalid length")

        with self._tx_cond:
            self._rx_bytes += 1
            self._rx_packets += 1
            self._tx_cond.notify()

        log.debug(f"RX: {HEX(p)}")
        data = struct.unpack('<6f45B', p)
        pos = data[:6]
        touch = data[6:-2]
        stat = data[-2]
        cksum = data[-1]
        cksum_calc = checksum(p[:-1])

        if cksum != cksum_calc:
            log.warning( f'packet checksum error: expected 0x{cksum_calc} received 0x{cksum}')
            return

        # convert float list to JointData
        self._pos = JointData(*pos)
        self._touch = TouchSensors.unpack(touch)

        log.debug("RX: {self._pos} : {self._touch}")


    def read_v2(
        self,
    ) -> None:
        p_type:Optional[bytes] = None
        p_len = 0
        p_sum = 0

        p_type = self._comm.read(1)

        if p_type is None:
            return

        # trigger TX thread as soon as first bytes detected
        with self._tx_cond:
            self._rx_bytes += 1
            self._rx_packets += 1
            self._tx_cond.notify()
            time.sleep(0)

        if p_type is None or len(p_type) != 1:
            raise ProtocolError("invalid/empty packet type")

        format = p_type[0]
        variant = format & 0xF

        if variant == 2:
            p_len = 38
        else:
            p_len = 71

        p = self._comm.read(p_len)

        log.debug(f"RX: {HEX(p)}")

        if p is None or len(p) != p_len:
            raise ProtocolError("invalid length")

        self._rx_bytes += len(p)

        p_sum = p[-1]
        # strip received checksum for checksum calc
        data = p[:-1]
        # reassemble header for checksum calc
        p = p_type + data

        log.debug(f"calculated checksum: {p_sum:x}")

        if p_sum != checksum(p):
            raise ProtocolError(
                f"checksum failed: received {p_sum} expected: {checksum(p)}"
            )

        # variant 0: [ (position, motor current), ... ]  | [ touch sensor, ... ] | hot/cold status
        # variant 1: [ (position, rotor velocity), ... ] | [ touch sensor, ... ] | hot/cold status
        # variant 2: [ (position, motor current), ... ] | [ rotor velocity, ... ] | hot/cold status

        self._motor_status = MotorHotStatus.unpack(data[-1])

        # each finger: [position,current,..] or [position,velocity,...]
        d = struct.unpack("<12h", data[:24])

        decode_position = lambda pos: [p * 150 / 32767 for p in pos]
        decode_current = lambda qv: [d * 0.540 / 7000 for d in qv]
        decode_velocity = lambda qv: [d * 3000 / 32767 for d in qv]

        self._position = JointData(*decode_position(d[::2]))
        qv = d[1::2]

        if variant == 0:
            self._current = JointData(*decode_current(qv))
            self._touch = TouchSensors.unpack(data[24 : 24 + 45])
        elif variant == 1:
            self._velocity = JointData(*decode_velocity(qv))
            self._touch = TouchSensors.unpack(data[24 : 24 + 45])
        elif variant == 2:
            # each finger: u16 rotor velocity
            self._current = JointData(*decode_current(qv))
            self._velocity = JointData(
                *decode_velocity(struct.unpack("<6H", data[24:-1]))
            )
        else:
            raise ProtocolError(f"Unsupported reply variant; {variant}")
