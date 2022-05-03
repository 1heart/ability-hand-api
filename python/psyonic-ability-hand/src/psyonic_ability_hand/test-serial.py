
from serial import Serial
from hand import Hand, JointData
from io import SerialIO

baud=460800

if __name__ == '__main__':
	comm = Serial("/dev/ttyS0", baud)

	hand = Hand(comm, slave_address=0x50)

	jd = JointData(0x01, 0x02, 0x03, 0x10, 0x20, 0x30)

	hand.position_command(jd)
