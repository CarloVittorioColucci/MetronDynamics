# Registers
WHO_AM_I = const(0x0F)
CTRL2_G = const(0x11)
CTRL1_XL = const(0x10)
CTRL10_C = const(0x19)
CTRL3_C = const(0x12)

# This is the start of the data registers for the Gyro and Accelerometer
# There are 12 Bytes in total starting at 0x22 and ending at 0x2D
OUTX_L_G = const(0x22)

STEP_COUNTER_L = const(0x4B)
STEP_COUNTER_H = const(0x4C)
TAP_SRC = const(0x1C)
TAP_CFG = const(0x58)
FUNC_SRC1 = const(0x53)
FUNC_SRC2 = const(0x54)
TAP_THS_6D = const(0x59)
FREE_FALL = const(0x5D)
WAKE_UP_THS = const(0x5B)
WAKE_UP_SRC = const(0x1B)
INT_DUR2 = const(0x5A)

# CONFIG DATA
NORMAL_MODE_104HZ = const(0x40)
NORMAL_MODE_208HZ = const(0x50)
PERFORMANCE_MODE_416HZ = const(0x60)
LOW_POWER_26HZ = const(0x02)
SET_FUNC_EN = const(0xBD)
RESET_STEPS = const(0x02)
TAP_EN_XYZ = const(0x8E)
TAP_THRESHOLD = const(0x02)
DOUBLE_TAP_EN = const(0x80)
DOUBLE_TAP_DUR = const(0x20)

# Sensitivity constants
ACCEL_SENSITIVITY_2G = 0.061 / 1000   # g/LSB for ±2g range
ACCEL_SENSITIVITY_4G = 0.122 / 1000   # g/LSB for ±4g range
ACCEL_SENSITIVITY_8G = 0.244 / 1000   # g/LSB for ±8g range
ACCEL_SENSITIVITY_16G = 0.488 / 1000  # g/LSB for ±16g range

GYRO_SENSITIVITY_125DPS = 4.375 / 1000   # dps/LSB for ±125 dps range
GYRO_SENSITIVITY_250DPS = 8.75 / 1000    # dps/LSB for ±250 dps range
GYRO_SENSITIVITY_500DPS = 17.5 / 1000    # dps/LSB for ±500 dps range
GYRO_SENSITIVITY_1000DPS = 35.0 / 1000   # dps/LSB for ±1000 dps range
GYRO_SENSITIVITY_2000DPS = 70.0 / 1000   # dps/LSB for ±2000 dps range


def twos_comp(val, bits=16):
    mask = 1 << (bits - 1)

    if val & mask:
        val &= ~mask
        val -= mask

    return val


class LSM6DS3:
    # NOTE: default address is 0x6B (SA0 pin high).
    # Use address=0x6A if SA0 is pulled low on your board.
    def __init__(self, i2c, address=0x6B, mode=NORMAL_MODE_104HZ):
        self.bus = i2c
        self.address = address
        self.mode = mode

        # Calibration offsets (in raw LSB, applied before sensitivity conversion)
        self._accel_offset = [0, 0, 0]  # ax, ay, az
        self._gyro_offset = [0, 0, 0]   # gx, gy, gz

        # Verify device is present by reading WHO_AM_I (expected 0x69 for LSM6DS3)
        who = self.bus.readfrom_mem(self.address, WHO_AM_I, 1)
        if who[0] not in (0x69, 0x6A):
            raise ValueError("LSM6DS3 not found at address 0x{:02X} (WHO_AM_I=0x{:02X})".format(
                self.address, who[0]))

        # Set gyro mode/enable
        self.bus.writeto_mem(self.address, CTRL2_G, bytearray([self.mode]))

        # Set accel mode/enable
        self.bus.writeto_mem(self.address, CTRL1_XL, bytearray([self.mode]))

        # Send the reset bit to clear the pedometer step count
        self.bus.writeto_mem(self.address, CTRL10_C, bytearray([RESET_STEPS]))

        # Enable sensor functions (Tap, Tilt, Significant Motion)
        self.bus.writeto_mem(self.address, CTRL10_C, bytearray([SET_FUNC_EN]))

        # Enable X Y Z Tap Detection
        self.bus.writeto_mem(self.address, TAP_CFG, bytearray([TAP_EN_XYZ]))

        # Enable Double tap
        self.bus.writeto_mem(self.address, WAKE_UP_THS,
                             bytearray([DOUBLE_TAP_EN]))

        # Set tap threshold
        self.bus.writeto_mem(self.address, TAP_THS_6D,
                             bytearray([TAP_THRESHOLD]))

        # Set double tap max time gap
        self.bus.writeto_mem(self.address, INT_DUR2,
                             bytearray([DOUBLE_TAP_DUR]))

    def _read_reg(self, reg, size):
        return self.bus.readfrom_mem(self.address, reg, size)

    def calibrate(self, samples=200):
        """
        Calibrate the sensor by averaging readings while the device is perfectly still.
        The computed offsets are stored internally and automatically applied to all
        subsequent get_readings() and get_readings_g() calls.

        For the accelerometer, the gravity component on the resting axis is preserved,
        so you don't need to worry about orientation — only the bias is removed.

        :param samples: Number of samples to average (default 200, ~2 seconds at 104Hz)
        """
        import time

        print("Calibrating... keep the sensor still.")

        ax_sum = ay_sum = az_sum = 0
        gx_sum = gy_sum = gz_sum = 0

        # Temporarily zero out offsets so raw values are read
        self._accel_offset = [0, 0, 0]
        self._gyro_offset = [0, 0, 0]

        for _ in range(samples):
            ax, ay, az, gx, gy, gz = self.get_readings()
            ax_sum += ax
            ay_sum += ay
            az_sum += az
            gx_sum += gx
            gy_sum += gy
            gz_sum += gz
            time.sleep_ms(10)

        # Average the samples
        ax_avg = ax_sum // samples
        ay_avg = ay_sum // samples
        az_avg = az_sum // samples
        gx_avg = gx_sum // samples
        gy_avg = gy_sum // samples
        gz_avg = gz_sum // samples

        # For the accelerometer, remove only the bias by preserving the 1g gravity component.
        # Determine which axis has gravity (closest to ±1g = ±16384 LSB for ±2g range)
        # and subtract the expected 1g from that axis only.
        ONE_G_LSB = 16393  # 1g in LSB for ±2g range (1 / 0.000061)
        for val, i in zip([ax_avg, ay_avg, az_avg], range(3)):
            if abs(val) > ONE_G_LSB * 0.7:  # axis is carrying gravity
                sign = 1 if val > 0 else -1
                self._accel_offset[i] = val - sign * ONE_G_LSB
            else:
                self._accel_offset[i] = val

        # For gyro, the offset at rest is pure bias — remove it entirely
        self._gyro_offset = [gx_avg, gy_avg, gz_avg]

        print("Calibration done.")
        print("Accel offsets (LSB): X:{}, Y:{}, Z:{}".format(*self._accel_offset))
        print("Gyro offsets  (LSB): X:{}, Y:{}, Z:{}".format(*self._gyro_offset))

    def get_readings(self):
        """Returns bias-corrected raw LSB values: ax, ay, az, gx, gy, gz"""

        # Read 12 bytes starting from 0x22. This covers the XYZ data for gyro and accel
        data = self._read_reg(OUTX_L_G, 12)

        gx = twos_comp((data[1] << 8) | data[0]) - self._gyro_offset[0]
        gy = twos_comp((data[3] << 8) | data[2]) - self._gyro_offset[1]
        gz = twos_comp((data[5] << 8) | data[4]) - self._gyro_offset[2]

        ax = twos_comp((data[7] << 8) | data[6]) - self._accel_offset[0]
        ay = twos_comp((data[9] << 8) | data[8]) - self._accel_offset[1]
        az = twos_comp((data[11] << 8) | data[10]) - self._accel_offset[2]

        return ax, ay, az, gx, gy, gz

    def get_readings_g(self, accel_sens=ACCEL_SENSITIVITY_2G, gyro_sens=GYRO_SENSITIVITY_250DPS):
        """Returns converted values in g (accel) and dps (gyro), with calibration applied"""
        ax, ay, az, gx, gy, gz = self.get_readings()
        return (ax * accel_sens, ay * accel_sens, az * accel_sens,
                gx * gyro_sens, gy * gyro_sens, gz * gyro_sens)

    def get_step_count(self):
        data = self._read_reg(STEP_COUNTER_L, 2)
        steps = twos_comp((data[1] << 8) | data[0])
        return steps

    def reset_step_count(self):
        self.bus.writeto_mem(self.address, CTRL10_C, bytearray([RESET_STEPS]))
        self.bus.writeto_mem(self.address, CTRL10_C, bytearray([SET_FUNC_EN]))

    def tilt_detected(self):
        tilt = self._read_reg(FUNC_SRC1, 1)
        return (tilt[0] >> 5) & 0b1

    def sig_motion_detected(self):
        sig = self._read_reg(FUNC_SRC1, 1)
        return (sig[0] >> 6) & 0b1

    def single_tap_detected(self):
        s = self._read_reg(TAP_SRC, 1)
        return (s[0] >> 5) & 0b1

    def double_tap_detected(self):
        d = self._read_reg(TAP_SRC, 1)
        return (d[0] >> 4) & 0b1

    def freefall_detected(self):
        fall = self._read_reg(WAKE_UP_SRC, 1)
        return fall[0] >> 5
