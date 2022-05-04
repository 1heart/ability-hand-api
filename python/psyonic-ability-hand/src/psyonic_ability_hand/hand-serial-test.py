from pyserial import Serial
from typing import cast, IO
from hand import Hand, JointData

port = "/dev/ttyUSB0"
baud = 460800

comm = cast(IO, Serial(port, baud))

hand = Hand(comm, slave_address=0x50)

jd = JointData(0, 0, 0, 0, 0, 0)

hand.position_command(jd)

(pos, cur, vel, status) = hand.read()

print(f"Pos:{pos}")
print(f"Current:{cur}")
print(f"Velocity:{vel}")
print(f"status:{status}")
