import RPi.GPIO as GPIO
import time
import serial

#GPIO.setmode(GPIO.BOARD)
#GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)

baud = 460800

print(f"opening port at {baud}")
ser=serial.Serial("/dev/ttyS0", baud)

tx=b"abcdefghijgklmnopqrtuvwxyz"
tx = tx+tx+tx+tx
n = len(tx)
bps_expected = int(baud / 10.0)
errors = 0
start = time.time()
while True:
    ser.write(tx)
    d = ser.read(n)
    end = time.time()
    dt = end-start
    start = end
    bps = int(n/dt)

    if d != tx:
        errors += 1

    print(f"{dt:0.6f}  B/s:{bps}/{bps_expected} Errors:{errors}",end='\r')
