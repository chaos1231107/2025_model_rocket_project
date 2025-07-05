import RPi.GPIO as GPIO
import time
import board
import busio
import adafruit_bmp280
import math
import os

i2c = busio.I2C(board.SCL, board.SDA)
bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=0x76)
bmp280.sea_level_pressure = 1013.25

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)

init_altitude = bmp280.altitude
cali_altitude = 0.0
prev_altitude = 0.0
lpf_calialtitude = 0.0
prev_lpf_cali_altitude = 0.0
last_time = time.time()
start_time = time.time()

p_error = 0.0
kalman_Q = 0.02
kalman_estimate = 0.0
prev_kalman_estimate = 0.0
prev_p_error = 0.0
kalman_R = 0.9
falling_count = 0
falling_threshold = 3
falling_velocity = 0.0
falling_velocity_threshold = 0.2
#falling_velocity_threshold = 300.0 
#falling_depth = - 0.32.
ejection_flag = False
fname = "altitude_kalman2.txt"
if not os.path.exists(fname):
    with open(fname, 'w') as f:
        pass

try:
     while True:
        now = time.time()
        dt = now - last_time
        elapsed = now - start_time
        cali_altitude = bmp280.altitude - init_altitude
        if cali_altitude < 0.00:
            cali_altitude = 0.01 
        last_time = now
        #cutoff frequency of bmp280
        fc = 23.0
        rc = 1.0 / (2*math.pi*fc)
        beta = dt / (rc + dt)
        #kalman filter
        p_error += kalman_Q
        K2 = p_error / (p_error + kalman_R)
        kalman_estimate = prev_kalman_estimate + K2 * (cali_altitude - prev_kalman_estimate)
        p_error = (1.0 - K2) * prev_p_error
        #print(f"altitude = {cali_altitude:.2f}"
        lpf_cali_altitude = beta * kalman_estimate + (1.0 - beta) * prev_lpf_cali_altitude
   
        if abs(lpf_cali_altitude - prev_lpf_cali_altitude) > 0.01:
            falling_velocity = (lpf_cali_altitude - prev_lpf_cali_altitude) / dt

        if falling_velocity <= -falling_velocity_threshold:
            falling_count += 1
            print(falling_count)
       
        if falling_count >= falling_threshold:
            if not ejection_flag:
                print(">>Deploy!")
                GPIO.output(17, GPIO.HIGH)
                time.sleep(0.2)
                GPIO.output(17, GPIO.LOW)
                ejection_flag = True
           
       
        print(f"Raw Altitude = {cali_altitude:.2f},LPF Altitude = {kalman_estimate:.2f}, LPF+KMF Altitude = {lpf_cali_altitude:.2f}")

        with open("altitude_kalman2.txt", 'a') as f:
            f.write(
                f"{elapsed:.3f}, {cali_altitude:.2f},{kalman_estimate:.2f}, {lpf_cali_altitude:.2f}\n"
            ) 
        prev_altitude = cali_altitude
        prev_kalman_estimate = kalman_estimate
        prev_p_error = p_error
        prev_lpf_cali_altitude = lpf_cali_altitude
        time.sleep(0.005)

except KeyboardInterrupt:
    print("00")
