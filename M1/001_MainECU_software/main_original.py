from machine import I2C
from lsm6ds3_original import LSM6DS3, NORMAL_MODE_104HZ
import time
from Qwiic_original import Qwiic

# qwiic = Qwiic()  # initializes I2C on the easyC/Qwiic pins automatically
i2c = I2C(0, scl=9, sda=8, freq=400000)
sensor = LSM6DS3(i2c, address=0x6B, mode=NORMAL_MODE_104HZ)
# sensor = LSM6DS3(qwiic.i2c, mode=NORMAL_MODE_104HZ)

ACCEL_SENSITIVITY = 0.061 / 1000  # g/LSB for ±2g range
GYRO_SENSITIVITY = 8.75 / 1000    # dps/LSB for ±250 dps range

while True:
    ax, ay, az, gx, gy, gz = sensor.get_readings()
    print("Accelerometer (g)\nX:{:.3f}, Y:{:.3f}, Z:{:.3f}\nGyro (dps)\nX:{:.3f}, Y:{:.3f}, Z:{:.3f}\n".format(
        ax * ACCEL_SENSITIVITY, ay * ACCEL_SENSITIVITY, az * ACCEL_SENSITIVITY,
        gx * GYRO_SENSITIVITY, gy * GYRO_SENSITIVITY, gz * GYRO_SENSITIVITY))
    time.sleep(0.1)
