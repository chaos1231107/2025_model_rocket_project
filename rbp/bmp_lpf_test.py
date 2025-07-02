import RPi.GPIO as GPIO
import time
import board
import busio
import adafruit_bmp280
import math
import os

i2c = busio.I2C(board.SCL, board.SDA)
bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=0x76)

bmp280.sea_level_pressure = 1031.25
init_altitude = bmp280.altitude
cali_altitude = 0.0
prev_altiude = 0.0
lpf_calialtitude = 0.0
prev_lpf_cali_altitude = 0.0
last_time = time.time()
start_time = time.time()

fname = "altitude_data.txt"
if not os.path.exists(fname):
    with open(fname, 'w') as f:
        pass

try:
     while True:
        now = time.time()
        dt = now - last_time
        elapsed = now - start_time
        cali_altitude = bmp280.altitude - init_altitude
        last_time = now
        #cutoff frequency of bmp280
        fc = 23.0
        rc = 1.0 / (2*math.pi*fc)
        beta = dt / (rc + dt)
        #print(f"altitude = {cali_altitude:.2f}"
        lpf_cali_altitude = beta * cali_altitude + (1 - beta) * prev_lpf_cali_altitude
        print(f"Raw Altitude = {cali_altitude:.2f}, LPF Altitude = {lpf_cali_altitude:.2f}")

        with open("altitude_data.txt", 'a') as f:
            f.write(
                f"{elapsed:.3f}, {cali_altitude:.2f}, {lpf_cali_altitude:.2f}\n"
            ) 
        prev_altitude = cali_altitude
        prev_lpf_cali_altitude = lpf_cali_altitude
        time.sleep(0.02)

except KeyboardInterrupt:
    print("00")
