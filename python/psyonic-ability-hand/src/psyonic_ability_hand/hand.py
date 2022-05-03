import asyncio
import array
import dataclasses
import struct
import time
from enum import Enum, IntEnum
from typing import Any, IO, Protocol, List, Optional
from dataclasses import dataclass
import numpy as np
import evdev

from loguru import logger as log



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


"""
Finger  | Index     | Middle    |  Ring     | Pinky     | ThumbF    | ThumbR
----
I: s0    14.2 - 74.6
M: s1
R: 
P:
T1:
T2:



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
            pressures.append(p)

        return PressureData(*pressures)


class HandException(Exception):
    pass

class RcvError(Exception):
    pass

class TxError(Exception):
    pass

class FingerPosition:
    @staticmethod
    def to_int16(degrees:float):
        return int( (degrees * 32767) / 150 )

    @staticmethod
    def from_int16(data:int):
        return (data * 150) / 32767.0

class FingerVelocity:
    @staticmethod
    def to_int16(degrees_per_second:float):
        return degrees_per_second * 32767 / 3000

class Finger:
    def __init__(self, gear_ratio):
        self.gear_ratio = gear_ratio


class Hand:
    def __init__(self, comm: IO, slave_address=None):
        self._comm = comm
        self._slave_address = slave_address

    def checksum(self, data:bytes):
        cksum = 0
        for d in data:
            cksum = cksum + d
        return (-cksum) & 0xFF


    def _txbuf(self) -> array.array:
        return array.array("B", [0] * 26)

    def write(self, data: array.array):
        data[-1] = self.checksum(data)
        ret = self._comm.write(data)
        if ret != len(data):
            raise HandException(f"write error: {ret}")
        return ret

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

    def control_v1(self, mode: Mode, jointdata: Optional[JointData]=None):
        """
        Writes JointData and returns current status
        """
        RXPACKET_SIZE = 71
        if mode != Mode.Query and jointdata != None:
            buf = array.array("B", struct.pack("<B6fB", mode, *jointdata.to_list(), 0))
            self.write(buf)

        packet = self._comm.read(RXPACKET_SIZE)

        if len(packet) != RXPACKET_SIZE:
            raise RcvError(f"only received {len(packet)} bytes")

        data = struct.unpack("<6f45BBB", packet)

        jd = JointData(*data[:6])
        pressure = PressureData.from_array(self.unpack(data[6:-2]))
        status = data[-2]
        checksum = data[-1]
        calculated_checksum = self.checksum(array.array('B', packet[:-1]))

        if checksum != calculated_checksum:
            raise RcvError("checksum failed")

        return (jd, pressure, status)


    def position_command(self, pos:JointData, variant:int=0x12) -> bytes:
        buf = struct.pack('BB', self._slave_address, variant) 
        for d in pos.to_list():
            p = int(d * 32767 / 150)
            buf += struct.pack('<H', p & 0xFFFF ) 

        buf += struct.pack( 'B', self.checksum(buf) )

        self._comm.write(buf)

    def read(self):
        p_type = self._comm.read():
        p_len = 0

        if (p_type & 0xF) == 2:
            p_len = 38
        else:
            p_len = 71

        p = self._comm.read(p_len)

        




class App:

    RUN = True



if __name__ == "__main__":


    from rich.console import Console, Group
    from rich.columns import Columns
    from rich.live import Live
    from rich.bar import Bar
    from rich.text import Text
    from rich.progress_bar import ProgressBar
    from rich.table import Table
    from rich.layout import Layout
    from psyonic_ability_hand.io import I2CIO
    import sys, termios, tty, select
    import threading, queue
    from decimal import *

    async def display(app):
        mode = 0

        console = Console()

        log.info("creating I2C connection to hand")
        hand = Hand(I2CIO())

        # log.info("setting grip open")
        hand.set_grip(Grip.Open)
        # time.sleep(6.0)
        # log.info("setting grip closed")
        # hand.set_grip(Grip.Handshake)
        # time.sleep(6.0)
        # log.info("setting grip pinched")
        # hand.set_grip(Grip.PinchGrasp)
        # time.sleep(6.0)

        start = time.time()
        errors = 0

        layout = Layout()

        layout.split(
            Layout(name="info", size=2),
            Layout(name="header", size=1),
            Layout(name="errors", size=1),
            Layout(name="table")
        )


        pos_input = None
        torque_input = None
        velocity_input = None

        layout["info"].update("Move fingers with a,z | s,x | d,c | f,v | gb, hn. Change mode with 'm'. ")
        layout["header"].update("")
        layout["errors"].update("")


        try:

            with Live(auto_refresh=False) as screen:
                while app.RUN and (t:=time.time() - start): #< 100:
                    data = None

                    while not app.q.empty():
                        key = app.q.get_nowait()

                        def update_joint_data(val, keys):
                            if key is not None and key in keys:
                               val += (-1 if key == keys[0] else 1)
                            return val

                        if key == 'm':
                            mode = (mode + 1) % 3
                            layout['header'].update( Text( f'Mode: {mode}'))
                        elif pos_input:
                            pos_input.Pinky = update_joint_data(pos_input.Pinky, ['a','z'])
                            pos_input.Ring = update_joint_data(pos_input.Ring, ['s','x'])
                            pos_input.Middle = update_joint_data(pos_input.Middle, ['d','c'])
                            pos_input.Index = update_joint_data(pos_input.Index, ['f','v'])
                            pos_input.ThumbFlexor = update_joint_data(pos_input.ThumbFlexor, ['g','b'])
                            pos_input.ThumbRotator = update_joint_data(pos_input.ThumbRotator, ['h','n'])


                    try:
                        data = hand.control_v1(Mode.PosControl, pos_input or JointData() )
                    except ( OSError, RcvError, TxError ):
                        errors += 1
                        continue


                    table = Table(padding=(0,1))
                    table.add_column("Item")
                    table.add_column("Requested")
                    table.add_column("Position")
                    table.add_column("Pressures")

                    if data is not None:
                        (_position,_pressure,_status) = data

                        if not pos_input:
                            pos_input = _position
        
                        requested = dataclasses.asdict(pos_input)

                        # return { 'name' : position, ... }
                        position = dataclasses.asdict(_position)
                        # returns { 'name' : [pressures,], ...}
                        pressure = dataclasses.asdict(_pressure)

                        for finger in position:
                            pres = pressure.get(finger)

                            requested_col = f"{requested[finger]:> 12.6f}"
                            position_col = f"{position[finger]:> 12.6f}"

                            pressure_col = ''
                            if pres:
                                pressure_col = Columns( [ f"{p:> 12d}" for p in pres.values() ], equal=True )

                            table.add_row(finger, requested_col, position_col, pressure_col)
                    else:
                        errors += 1    

                    layout["errors"].update(f"Errors: {errors}")
                    layout['table'].update(table)
                    screen.update(layout, refresh=True)
                    await asyncio.sleep(.100)

                app.RUN = False

        except Exception as e:
            print(e)
            raise
        finally:
            pass
            # termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)



    def get_input(app):
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            fds = ([sys.stdin], [], [])
            ch = None
            while app.RUN: 
                if select.select(*fds, 0.010) == fds:
                    ch = sys.stdin.read(1)
                    app.loop.call_soon_threadsafe(app.q.put_nowait, ch)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            app.RUN=False



    async def main(app):
        await display(app)

    app = App()
    app.q = asyncio.Queue()
    app.loop = asyncio.get_event_loop()
    app.position = JointData()
    app.velocity = JointData()
    app.initialized = False

    input_thread = threading.Thread(target=get_input, args=(app,) )
    input_thread.start()

    try:
        app.loop.run_until_complete(main(app))        
    except (Exception,KeyboardInterrupt):
        app.RUN = False
        input_thread.join()
        print("exiting")
        