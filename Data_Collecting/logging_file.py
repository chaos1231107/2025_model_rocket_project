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

def deploy_parachute(ejection_flag):
    if not ejection_flag.value:
        GPIO.output(17, GPIO.HIGH)
        print(">> Parachute Deployed")
        time.sleep(0.2)
        GPIO.output(17, GPIO.LOW)
        ejection_flag.value = True

def stable_calibration(mpu, duration=3):
    print(f"Stabilizing MPU6050 Sensor for {duration}s")
    start = time.time()
    roll_list, pitch_list, yaw_list = [], [], []
    while time.time() - start < duration:
        accel = mpu.get_accel_data()
        gyro = mpu.get_gyro_data()
        roll = math.atan2(accel['y'], accel['z']) * 180 / math.pi
        pitch = math.atan2(-accel['x'], math.sqrt(accel['y']**2 + accel['z']**2)) * 180 / math.pi
        yaw = gyro['z']
        roll_list.append(roll)
        pitch_list.append(pitch)
        yaw_list.append(yaw)
        time.sleep(0.01)
    return (sum(roll_list) / len(roll_list), sum(pitch_list) / len(pitch_list), sum(yaw_list) / len(yaw_list))

def logging_process(log_queue):
    f_gyro = open('gyro.txt', 'a')
    f_alt = open('alt.txt', 'a')
    f_comm = open('comm_data.txt', 'a')
    try:
        while True:
            entry = log_queue.get()
            if entry == "EXIT":
                break
            file_tag, message = entry
            if file_tag == 'gyro':
                f_gyro.write(message + '\n')
                f_gyro.flush()
                os.fsync(f_gyro.fileno())

            elif file_tag == 'alt':
                f_alt.write(message + '\n')
                f_alt.flush()
                os.fsync(f_alt.fileno())

            elif file_tag == 'comm':
                f_comm.write(message + '\n')
                f_comm.flush()
                os.fsync(f_comm.fileno())

    finally:
        f_gyro.close()
        f_alt.close()
        f_comm.close()
        print(">> Logging process terminated.")

def comm_process(roll, pitch, yaw, altitude, ejection_flag, lock, log_queue):
    ser = serial.Serial('/dev/ttyS0', 19200, timeout=0.2)
    start_time = time.time()
    deploy_time = None
    try:
        while True:
            now = time.time()
            elapsed = now - start_time
            if ser.in_waiting:
                msg = ser.readline().decode(errors='ignore').strip()
                if msg.lower() == 'd':
                    print(">> Command Received: DEPLOY")
                    deploy_parachute(ejection_flag)
            tx = f"<<{roll.value:.2f},{pitch.value:.2f},{yaw.value:.2f},{altitude.value:.2f}>>"
            ser.write((tx + '\n').encode())
            print(f'send -> Roll:{roll.value:.2f}, Pitch:{pitch.value:.2f}, Yaw:{yaw.value:.2f} Alt:{altitude.value:.2f}')
            log_queue.put(('comm', f'Elapsed : {elapsed:.2f}, Roll : {roll.value:.2f}, Pitch: {pitch.value:.2f}, Yaw: {yaw.value:.2f}, Altitude: {altitude.value:.2f}'))
            if ejection_flag.value:
                if deploy_time is None:
                    deploy_time = time.time()
                elif time.time() - deploy_time > 5:
                    break
            time.sleep(0.07)
    finally:
        ser.close()
        print('Communication process closed!')

def sensor_process(roll, pitch, yaw, altitude, ejection_flag, lock, log_queue):
    i2c = busio.I2C(board.SCL, board.SDA)
    bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=0x76)
    bmp280.sea_level_pressure = 1013.25
    time.sleep(1.0)
    init_alt = bmp280.altitude
    mpu = mpu6050(0x68)
    init_roll, init_pitch, yaw_bias = stable_calibration(mpu)
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
        p[1][1] += Q_bias * dt
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

    kalman_Q, kalman_R = 0.02, 2.5
    kalman_estimate = 0.0
    prev_kalman_estimate = 0.0
    p_error = 1.0
    prev_p_error = 1.0
    lpf_cali_altitude = 0.0
    prev_lpf_cali_altitude = 0.0
    falling_count = 0
    falling_threshold = 3
    #Testing Value
    deploy_alt = 0.7
    fall_speed_limit = 0.4
    angle_yaw = 0.0
    prev_lpf_roll = 0.0
    prev_lpf_pitch = 0.0
    prev_lpf_yaw = 0.0
    print("System Stabilizing...3s")
    time.sleep(3)
    last_time = time.time()
    last_debug_time = time.time()
    start_time = time.time()

    try:
        while not ejection_flag.value:
            now = time.time()
            dt = now - last_time
            last_time = now
            elapsed = now - start_time
            fc = 24.22
            rc = 1.0 / (2 * math.pi * fc)
            alpha = dt / (rc + dt)

            cali_alt = bmp280.altitude - init_alt
            cali_alt = max(0.0, cali_alt)
            predicted_estimate = prev_kalman_estimate
            predicted_p_error = prev_p_error + kalman_Q
            kalman_gain = predicted_p_error / (predicted_p_error + kalman_R)
            kalman_estimate = predicted_estimate + kalman_gain * (cali_alt - predicted_estimate)
            p_error = (1 - kalman_gain) * predicted_p_error
            rc_bmp = 1.0 / (2 * math.pi * 7.0)
            beta = dt / (rc_bmp + dt)
            lpf_cali_altitude = beta * kalman_estimate + (1 - beta) * prev_lpf_cali_altitude
            fall_speed = (lpf_cali_altitude - prev_lpf_cali_altitude) / dt if dt > 0 else 0.0
            falling_count = falling_count + 1 if fall_speed <= -fall_speed_limit else 0

            accel = mpu.get_accel_data()
            gyro = mpu.get_gyro_data()
            angle_roll = math.atan2(accel['y'], accel['z']) * 180 / math.pi
            angle_pitch = math.atan2(-accel['x'], math.sqrt(accel['y']**2 + accel['z']**2)) * 180 / math.pi
            relative_roll = angle_roll - init_roll
            relative_pitch = angle_pitch - init_pitch
            gyro_x = gyro['x'] if abs(gyro['x']) <= 1000 else 0.0
            gyro_y = gyro['y'] if abs(gyro['y']) <= 1000 else 0.0
            gyro_z = gyro['z'] - yaw_bias
            gyro_z = gyro_z if abs(gyro_z) <= 1000 else 0.0

            angle_yaw = (angle_yaw + gyro_z * dt) % 360
            kal_roll = kalman_filter('x', relative_roll, gyro_x, dt)
            kal_pitch = kalman_filter('y', relative_pitch, gyro_y, dt)
            kal_yaw = kalman_filter('z', angle_yaw, gyro_z, dt)
            lpf_roll = alpha * kal_roll + (1 - alpha) * prev_lpf_roll
            lpf_pitch = alpha * kal_pitch + (1 - alpha) * prev_lpf_pitch
            lpf_yaw = alpha * kal_yaw + (1 - alpha) * prev_lpf_yaw
            roll.value = lpf_roll
            pitch.value = lpf_pitch
            yaw.value = lpf_yaw
            altitude.value = lpf_cali_altitude

            if now - last_debug_time > 0.5:
                print(f"Alt: raw={bmp280.altitude:.2f}, cali={cali_alt:.2f}, kalman={kalman_estimate:.2f}, lpf={lpf_cali_altitude:.2f}")
                print(f"Relative angles: Roll={lpf_roll:.2f}°, Pitch={lpf_pitch:.2f}°, Yaw={lpf_yaw:.2f}°")
                print(f"Fall speed: {fall_speed:.3f}, Count: {falling_count}")
                last_debug_time = now
            if (abs(lpf_roll) > 45.0 or abs(lpf_pitch) > 45.0) and lpf_cali_altitude >= deploy_alt:
                print(f">> Angle trigger: Roll={lpf_roll:.1f}, Pitch={lpf_pitch:.1f}")
                deploy_parachute(ejection_flag)
            if falling_count >= falling_threshold and lpf_cali_altitude >= deploy_alt:
                print(f">> Fall trigger: Count={falling_count}, Speed={fall_speed:.3f}")
                deploy_parachute(ejection_flag)
            log_queue.put(('gyro', f'Elapsed : {elapsed:.2f}, Roll : {roll.value:.2f}, Pitch : {pitch.value:.2f}, Yaw : {yaw.value:.2f}'))
            log_queue.put(('alt', f'Elapsed : {elapsed:.2f}, Alt : {altitude.value:.2f}'))
            prev_lpf_roll = lpf_roll
            prev_lpf_pitch = lpf_pitch
            prev_lpf_yaw = lpf_yaw
            prev_kalman_estimate = kalman_estimate
            prev_p_error = p_error
            prev_lpf_cali_altitude = lpf_cali_altitude
            time.sleep(0.05)
    finally:
        print('Sensor process closed!')

if __name__ == "__main__":
    try:
        roll = multiprocessing.Value('d', 0.0)
        pitch = multiprocessing.Value('d', 0.0)
        yaw = multiprocessing.Value('d', 0.0)
        altitude = multiprocessing.Value('d', 0.0)
        ejection_flag = multiprocessing.Value(c_bool, False)
        lock = multiprocessing.Lock()

        log_queue = multiprocessing.Queue()
        log_proc = multiprocessing.Process(target=logging_process, args=(log_queue,))
        log_proc.start()
        p1 = multiprocessing.Process(target=sensor_process, args=(roll, pitch, yaw, altitude, ejection_flag, lock, log_queue))
        p2 = multiprocessing.Process(target=comm_process, args=(roll, pitch, yaw, altitude, ejection_flag, lock, log_queue))
        p1.start()
        p2.start()
        p1.join()
        p2.join()

        log_queue.put("EXIT")
        log_proc.join()
    except KeyboardInterrupt:
        print(">> Stopped by user")
    finally:
        GPIO.cleanup()
