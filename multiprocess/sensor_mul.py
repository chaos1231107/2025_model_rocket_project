import time, math, serial, os, board, busio
import RPi.GPIO as GPIO
import multiprocessing
from ctypes import c_bool
from mpu6050 import mpu6050
import adafruit_bmp280

# Setup GPIO for parachute
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.output(17, GPIO.LOW)

# Deploy parachute if not already deployed
def deploy_parachute(ejection_flag):
    if not ejection_flag.value:
        GPIO.output(17, GPIO.HIGH)
        print(">> Parachute Deployed")
        time.sleep(0.2)
        GPIO.output(17, GPIO.LOW)
        ejection_flag.value = True

# Communication process
def comm_process(roll, pitch, yaw, altitude, ejection_flag, lock):
    ser = serial.Serial('/dev/ttyS0', 19200, timeout=0.3)
    while True:
        if ser.in_waiting:
            msg = ser.readline().decode(errors='ignore').strip()
            if msg.lower() == 'd':
                print(">> Command Received: DEPLOY")
                deploy_parachute(ejection_flag)

        # Send filtered sensor values
        tx = f"ROLL:{roll.value:.1f}, PITCH:{pitch.value:.1f}, YAW:{yaw.value:.1f} ALT:{altitude.value:.1f}"
        ser.write((tx + '\n').encode())
        print(f'send -> Roll:{roll.value:.1f}, Pitch:{pitch.value:.1f}, Yaw:{yaw.value:.1f} Alt:{altitude.value:.1f}')
        time.sleep(0.05)

# Sensor process: read sensors, filter, check conditions
def sensor_process(roll, pitch, yaw, altitude, ejection_flag, lock):
    i2c = busio.I2C(board.SCL, board.SDA)
    bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=0x76)
    bmp280.sea_level_pressure = 1013.25
    time.sleep(1.0)
    init_alt = bmp280.altitude
    mpu = mpu6050(0x68)

    # Kalman filter setup for angles
    kalman_angle = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    bias = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    P = {axis: [[1.0, 0.0], [0.0, 1.0]] for axis in 'xyz'}
    Q_angle, Q_bias, R_measure = 0.0025, 0.000025, 0.25

    def kalman_filter(axis, new_angle, new_rate, dt):
        angle = kalman_angle[axis]
        b = bias[axis]
        p = P[axis]
        rate = new_rate - b
        angle += rate * dt

        p[0][0] += dt * (dt*p[1][1] - p[0][1] - p[1][0] + Q_angle)
        p[0][1] -= dt * p[1][1]
        p[1][0] -= dt * p[1][1]
        p[1][1] += Q_angle * dt

        S = p[0][0] + R_measure
        K = [p[0][0] / S, p[1][0] / S]
        y = new_angle - angle

        angle += K[0] * y
        b += K[1] * y

        p00, p01 = p[0][0], p[0][1]
        p[0][0] -= K[0] * p00
        p[0][1] -= K[0] * p01
        p[1][0] -= K[1] * p00
        p[1][1] -= K[1] * p01

        kalman_angle[axis] = angle
        bias[axis] = b
        return angle

    # Altitude Kalman filter variables
    kalman_Q, kalman_R = 0.02, 0.9
    kalman_estimate = 0.0
    prev_kalman_estimate = 0.0
    p_error = 1.0
    prev_p_error = 1.0

    lpf_cali_altitude = 0.0
    prev_lpf_cali_altitude = 0.0

    falling_count = 0
    falling_threshold = 3
    fall_speed_limit = 0.2

    # Angle memory
    prev = {'yaw': 0.0, 'kal_roll': 0.0, 'kal_pitch': 0.0, 'kal_yaw': 0.0}
    last_time = time.time()
    last_debug_time = time.time()

    while not ejection_flag.value:
        now = time.time()
        dt = now - last_time
        last_time = now

        # Low pass filter alpha
        alpha = dt / (1 / (2 * math.pi * 24.22) + dt)

        # 현재 보정된 고도
        cali_alt = bmp280.altitude - init_alt
        if cali_alt < 0.0:
            cali_alt = 0.1
        predicted_estimate = prev_kalman_estimate
        predicted_p_error = prev_p_error + kalman_Q

        kalman_gain = predicted_p_error / (predicted_p_error + kalman_R)
        kalman_estimate = predicted_estimate + kalman_gain * (cali_alt - predicted_estimate)
        p_error = (1 - kalman_gain) * predicted_p_error

        # Low-pass filtered altitude
        rc_bmp = 1.0 / (2 * math.pi * 23.0)
        beta = dt / (rc_bmp + dt)
        lpf_cali_altitude = beta * kalman_estimate + (1 - beta) * prev_lpf_cali_altitude
        fall_speed = (lpf_cali_altitude - prev_lpf_cali_altitude) / dt if dt > 0 else 0.0

        # Count falling
        if fall_speed < -fall_speed_limit:
            falling_count += 1
        else:
            falling_count = 0

        # Get sensor data
        accel = mpu.get_accel_data()
        gyro = mpu.get_gyro_data()

        raw_roll = math.atan2(accel['y'], accel['z']) * 180 / math.pi
        raw_pitch = math.atan2(-accel['x'], math.sqrt(accel['y']**2 + accel['z']**2)) * 180 / math.pi

        gyro_x = gyro['x'] if abs(gyro['x']) < 1000 else 0.0
        gyro_y = gyro['y'] if abs(gyro['y']) < 1000 else 0.0
        gyro_z = gyro['z'] if abs(gyro['z']) < 1000 else 0.0

        kal_roll = kalman_filter('x', raw_roll, gyro_x, dt)
        kal_pitch = kalman_filter('y', raw_pitch, gyro_y, dt)
        kal_yaw = kalman_filter('z', prev['yaw'], gyro_z, dt)

        # Low-pass filter angles
        lpf_roll = alpha * kal_roll + (1 - alpha) * prev['kal_roll']
        lpf_pitch = alpha * kal_pitch + (1 - alpha) * prev['kal_pitch']
        lpf_yaw = alpha * kal_yaw + (1 - alpha) * prev['kal_yaw']

        # Save to shared memory
        roll.value = lpf_roll
        pitch.value = lpf_pitch
        yaw.value = lpf_yaw
        altitude.value = lpf_cali_altitude

        if now - last_debug_time > 0.5:
            print(f"Alt: raw={bmp280.altitude:.2f}, cali={cali_alt:.2f}, kalman={kalman_estimate:.2f}, lpf={lpf_cali_altitude:.2f}")
            print(f"Fall speed: {fall_speed:.3f}, Count: {falling_count}")
            last_debug_time = now

        # Trigger parachute if needed
        if abs(lpf_roll) > 35.0 or abs(lpf_pitch) > 30.0:
            print(f">> Angle trigger: Roll={lpf_roll:.1f}, Pitch={lpf_pitch:.1f}")
            deploy_parachute(ejection_flag)
        if falling_count >= falling_threshold:
            print(f">> Fall trigger: Count={falling_count}, Speed={fall_speed:.3f}")
            deploy_parachute(ejection_flag)

        prev.update({'kal_roll': lpf_roll, 'kal_pitch': lpf_pitch, 'kal_yaw': lpf_yaw})
        prev['yaw'] += gyro_z * dt

        prev_kalman_estimate = kalman_estimate
        prev_p_error = p_error
        prev_lpf_cali_altitude = lpf_cali_altitude

        time.sleep(0.02)

# Run processes
if __name__ == "__main__":
    try:
        roll = multiprocessing.Value('d', 0.0)
        pitch = multiprocessing.Value('d', 0.0)
        yaw = multiprocessing.Value('d', 0.0)
        altitude = multiprocessing.Value('d', 0.0)
        ejection_flag = multiprocessing.Value(c_bool, False)
        lock = multiprocessing.Lock()

        p1 = multiprocessing.Process(target=sensor_process, args=(roll, pitch, yaw, altitude, ejection_flag, lock))
        p2 = multiprocessing.Process(target=comm_process, args=(roll, pitch, yaw, altitude, ejection_flag, lock))

        p1.start()
        p2.start()
        p1.join()
        p2.join()

    except KeyboardInterrupt:
        print(">> Stopped by user")

    finally:
        GPIO.cleanup()
