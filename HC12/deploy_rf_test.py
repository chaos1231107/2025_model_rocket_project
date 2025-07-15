import time
import string
import random
import serial
import math
import RPi.GPIO as GPIO
from mpu6050 import mpu6050

mpu = mpu6050(0x68)
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
ejection_flag = False

ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
print("Start Test")
dt = 0.1
yaw = 0.0

while True:
    echo = ''
    accel = mpu.get_accel_data()
    gyro = mpu.get_gyro_data()

    roll = math.atan2(accel['y'] , accel['z']) * 180 / math.pi
    pitch = math.atan2(-accel['x'], math.sqrt(accel['y'] ** 2 + accel['z'] ** 2))
    gyro_z = gyro['z'] 
    yaw += gyro_z * dt
    yaw %= 360
   # if yaw > 360:
       # yaw -= 360

    msg = f"roll : {roll:.2f}, pitch : {pitch:.2f}, yaw : {yaw:.2f}"
    ser.write((msg + '\n').encode())
    print("send : ", msg)

    if ser.in_waiting:
        echo  = ser.readline().decode(errors='ignore').strip()
        print(f"Recieved : {echo}")
    if echo == "DEPLOY" and not ejection_flag:
        GPIO.output(17, GPIO.HIGH)
        print(">> DEPLOY SIGNAL RECIEVED!")
        time.sleep(0.2)
        GPIO.output(17, GPIO.LOW)
        ejection_flag = True

    time.sleep(0.5)
