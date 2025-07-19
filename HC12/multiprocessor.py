import multiprocessing
import time
import math
import serial
import os
import board
import busio
import RPi.GPIO as GPIO
from mpu6050 import mpu6050
import adafruit_bmp280

def sensor_process(ejection_flag):
    i2c = busio.I2C(board.SCL, board.SDA)
    bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=0x76)
    bmp280.sea_level_pressure = 1013.25
    mpu = mpu6050(0x68)

    init_alt = bmp280.altitude
    start_time = time.time()
    last_time = time.time()

    while not ejection_flag.value:
        now = time.time()
        dt = now - last_time
        last_time = now
        elapsed = now - start_time

        # Sensor readings
        accel = mpu.get_accel_data()
        gyro = mpu.get_gyro_data()
        roll = math.atan2(accel['y'], accel['z']) * 180 / math.pi
        pitch = math.atan2(-accel['x'], math.sqrt(accel['y']**2 + accel['z']**2)) * 180 / math.pi
        yaw = gyro['z']
        altitude = bmp280.altitude - init_alt

        print(f"[SENSOR] {elapsed:.2f}s | Roll: {roll:.1f} | Pitch: {pitch:.1f} | Yaw: {yaw:.1f} | Alt: {altitude:.2f}")

        with open("sensor_log.txt", "a") as f:
            f.write(f"{elapsed:.2f},{roll:.2f},{pitch:.2f},{yaw:.2f},{altitude:.2f}\n")

        time.sleep(0.005)

def comms_process(ejection_flag):
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(17, GPIO.OUT)
    GPIO.output(17, GPIO.LOW)

    ser = serial.Serial("/dev/ttyS0", 19200, timeout=1)
    print("[COMMS] Listening for DEPLOY command...")

    while not ejection_flag.value:
        if ser.in_waiting:
            msg = ser.readline().decode(errors='ignore').strip()
            print("[COMMS] Received:", msg)
            if msg == "DEPLOY":
                GPIO.output(17, GPIO.HIGH)
                print(">> DEPLOY SIGNAL RECEIVED - Parachute Deployed")
                time.sleep(0.2)
                GPIO.output(17, GPIO.LOW)
                ejection_flag.value = True
        time.sleep(0.1)

    GPIO.cleanup()

if __name__ == "__main__":
    try:
        ejection_flag = multiprocessing.Value('b', False)

        sensor = multiprocessing.Process(target=sensor_process, args=(ejection_flag,))
        comms = multiprocessing.Process(target=comms_process, args=(ejection_flag,))

        sensor.start()
        comms.start()

        sensor.join()
        comms.join()

    except KeyboardInterrupt:
        print("Program interrupted.")
        GPIO.cleanup()
