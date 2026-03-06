from lsm6ds3 import LSM6DS3, NORMAL_MODE_104HZ
from Qwiic import Qwiic
import time
import sys
import uselect

qwiic = Qwiic()  # uses GPIO9 (SCL) and GPIO8 (SDA) by default
sensor = LSM6DS3(qwiic.i2c, mode=NORMAL_MODE_104HZ)  # address defaults to 0x6B

# Set up non-blocking stdin poll
poll = uselect.poll()
poll.register(sys.stdin, uselect.POLLIN)

print("Running. Press 'c' at any time to calibrate (keep sensor still when doing so).")

while True:
    # Check for keyboard input without blocking
    if poll.poll(0):  # 0ms timeout = non-blocking
        key = sys.stdin.read(1)
        if key == 'c':
            sensor.calibrate(samples=500)

    ax, ay, az, gx, gy, gz = sensor.get_readings_g()
    print("Accelerometer (g)\nX:{:.3f}, Y:{:.3f}, Z:{:.3f}\nGyro (dps)\nX:{:.3f}, Y:{:.3f}, Z:{:.3f}\n".format(
        ax, ay, az, gx, gy, gz))
    time.sleep(0.5)
