import asyncio
import time
import dataclasses
import sys, termios, tty, select
import threading, queue

from typing import cast

from serial import Serial


from rich.style import Style
from rich.console import Console, Group
from rich.columns import Columns
from rich.live import Live
from rich.bar import Bar
from rich.text import Text
from rich.progress_bar import ProgressBar
from rich.table import Table
from rich.layout import Layout
from decimal import *
from loguru import logger as log

from psyonic_ability_hand.hand import Hand, ProtocolError, JointData, Grip, MockComm, ReplyType


class App:
    RUN:bool = True
    q: asyncio.Queue
    loop: asyncio.AbstractEventLoop
    position: JointData
    velocity: JointData
    initialized: bool

async def display(app):
    mode = 0

    console = Console()

    io = MockComm()
#    io = Serial("/dev/ttyUSB0")
    hand = Hand(io)

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
    mode = 0

    layout = Layout()

    layout.split(
        Layout(name="info", size=2),
        Layout(name="header", size=1),
        Layout(name="errors", size=1),
        Layout(name="table"),
    )

    pos_input = None
    torque_input = JointData()
    velocity_input = JointData(50, 50, 50, 50, 50, 50)

    layout["info"].update(
        "Move fingers with a,z | s,x | d,c | f,v | gb, hn. Change mode with 'm'. "
    )
    layout["header"].update("")
    layout["errors"].update("")

    active_column = Style(color='cyan')

    try:

        hand.start()

        with Live(layout, auto_refresh=False, screen=True) as live:
            while app.RUN and (t := time.time() - start):  # < 100:
                data = None

                if not app.q.empty():
                    while not app.q.empty():
                        key = app.q.get_nowait()

                        if not pos_input:
                            break

                        def update_joint_data(val, keys):
                            if key is not None and key in keys:
                                val += -1 if key == keys[0] else 1
                            return val

                        if key == "m":
                            mode = (mode + 1) % 4
                        elif pos_input:
                            pos_input.Pinky = update_joint_data(
                                pos_input.Pinky, ["a", "z"]
                            )
                            pos_input.Ring = update_joint_data(
                                pos_input.Ring, ["s", "x"]
                            )
                            pos_input.Middle = update_joint_data(
                                pos_input.Middle, ["d", "c"]
                            )
                            pos_input.Index = update_joint_data(
                                pos_input.Index, ["f", "v"]
                            )
                            pos_input.ThumbFlexor = update_joint_data(
                                pos_input.ThumbFlexor, ["g", "b"]
                            )
                            pos_input.ThumbRotator = update_joint_data(
                                pos_input.ThumbRotator, ["h", "n"]
                            )

                    log.debug(f"sending position update: {pos_input}")
                    hand.set_position(pos_input)

                table = Table(padding=(0, 1))
                table.add_column("Item")
                table.add_column("Position", header_style=(active_column if mode==0 else None) )
                table.add_column("Velocity", header_style=(active_column if mode==1 else None) )
                table.add_column("Torque", header_style=(active_column if mode==2 else None))
                table.add_column("PWM", header_style=(active_column if mode==3 else None))
                table.add_column("Pressures")

                if not pos_input:
                    log.info(f"updating position input to {hand.position}")
                    pos_input = hand.position

                requested = dataclasses.asdict(pos_input)
                position = dataclasses.asdict(hand.position)
                pressure = dataclasses.asdict(hand.touch)
                velocity = dataclasses.asdict(hand.velocity)

                for finger in position:
                    pres = pressure.get(finger)

                    position_col = f"{requested[finger]:> 12.6f} | {position[finger]:> 12.6f}"

                    velocity_col = ''
                    torque_col = ''
                    pwm_col = ''

                    pressure_col = ""
                    if pres:
                        pressure_col = Columns(
                            [f"{p:> 12.3f}" for p in pres.values()], equal=True
                        )

                    table.add_row(
                        finger, position_col, velocity_col, torque_col, pwm_col, pressure_col
                    )
            # else:
            #     errors += 1
            #     layout["errors"].update(f"Errors: {errors}")

                layout["table"].update(table)
                live.update(layout, refresh=True) # screen.update(layout, refresh=True)
                await asyncio.sleep(0.030)

            app.RUN = False

    except Exception as e:
        log.exception("error")
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
        app.RUN = False

async def main(app):
    await display(app)

app = App()
app.q = asyncio.Queue()
app.loop = asyncio.get_event_loop()
app.position = JointData()
app.velocity = JointData()
app.initialized = False

input_thread = threading.Thread(target=get_input, args=(app,))
input_thread.start()

try:
    app.loop.run_until_complete(main(app))
except (Exception, KeyboardInterrupt):
    app.RUN = False
    input_thread.join()
    print("exiting")
