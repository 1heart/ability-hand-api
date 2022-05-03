
from serial import Serial
from typing import cast, IO
from hand import Hand, JointData
from io import SerialIO, IOBase

port="/dev/ttyUSB0"
baud=460800

if __name__ == '__main__':
	comm = cast(IO, Serial(port, baud))

	hand = Hand(comm, slave_address=0x50)

	jd = JointData(0, 0, 0, 0, 0, 0)

	hand.position_command(jd)

	(pos, cur, vel, status) = hand.read()

	print(f"Pos:{pos}")
	print(f"Current:{cur}")
	print(f"Velocity:{vel}")
	print(f"status:{status}")
