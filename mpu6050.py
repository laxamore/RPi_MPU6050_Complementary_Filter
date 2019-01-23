#!/usr/bin/python3

import smbus
import math

def read_word(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg+1)
    value = (h << 8) + l
    return value

def read_word_2c(reg):
    val = read_word(reg)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

bus = smbus.SMBus(1)
address = 0x68
power_mgmt_1 = 0x6b

bus.write_byte_data(address, power_mgmt_1, 0)

pitch = 0
roll = 0

def ComplementaryFilter(accData, gyroData, p, r):
    dt = 0.01 #Sample Time 10ms
    ACCELEROMETER_SENSITIVITY = 8192.0
    GYROSCOPE_SENSITIVITY = 65.536

    pitchAcc = 0.0
    rollAcc = 0.0
    
    p += (gyroData[0] / GYROSCOPE_SENSITIVITY) * dt
    r -= (gyroData[1] / GYROSCOPE_SENSITIVITY) * dt
    
    forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2])
    if forceMagnitudeApprox > 8192 and forceMagnitudeApprox < 32768:
        pitchAcc = math.atan2(float(accData[1]), float(accData[2])) * 180 / math.pi;
        p = p * 0.98 + pitchAcc * 0.02;
        rollAcc = math.atan2(float(accData[0]), float(accData[2])) * 180 / math.pi;
        r = r * 0.98 + rollAcc * 0.02;
    
    return p, r

while True:
    acceleration_xout = read_word_2c(0x3b)
    acceleration_yout = read_word_2c(0x3d)
    acceleration_zout = read_word_2c(0x3f)
    
    gyro_xout = read_word_2c(0x43)
    gyro_yout = read_word_2c(0x45)
    gyro_zout = read_word_2c(0x47)

    pitch, roll = ComplementaryFilter([acceleration_xout, acceleration_yout, acceleration_zout], [gyro_xout, gyro_yout, gyro_zout], pitch, roll)

    print(pitch, roll)
