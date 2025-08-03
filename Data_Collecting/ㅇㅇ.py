import time, math, os, serial, board, busio
import RPi.GPIO as GPIO
import threading
import multiprocessing
from ctypes import c_bool
from mpu6050 import mpu6050
import adafruit_bmp280
import queue

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.output(17, GPIO.LOW)

# Shared state
roll = multiprocessing.Value('d', 0.0)
pitch = multiprocessing.Value('d', 0.0)
yaw = multiprocessing.Value('d', 0.0)
altitude = multiprocessing.Value('d', 0.0)
ejection_flag = multiprocessing.Value(c_bool, False)

roll_lock = threading.Lock()
alt_lock = threading.Lock()

log_queue = queue.Queue()

# Parachute deploy
def deploy_parachute():
    if not ejection_flag.value:
        GPIO.output(17, GPIO.HIGH)
        print(">> Parachute Deployed")
        time.sleep(0.2)
        GPIO.output(17, GPIO.LOW)
        ejection_flag.value = True

# Logging thread
def logging_thread():
    with open('gyro.txt', 'a') as f_gyro, open('alt.txt', 'a') as f_alt, open('comm_data.txt', 'a') as f_comm:
        while True:
            try:
                file_tag, message = log_queue.get(timeout=0.5)
                if file_tag == 'EXIT':
                    break
                if file_tag == 'gyro':
                    f_gyro.write(message + '\n'); f_gyro.flush(); os.fsync(f_gyro.fileno())
                elif file_tag == 'alt':
                    f_alt.write(message + '\n'); f_alt.flush(); os.fsync(f_alt.fileno())
                elif file_tag == 'comm':
                    f_comm.write(message + '\n'); f_comm.flush(); os.fsync(f_comm.fileno())
            except queue.Empty:
                continue
    print(">> Logging thread terminated.")

# Communication thread
def comm_thread():
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
                    deploy_parachute()

            with roll_lock, alt_lock:
                tx = f"<<{roll.value:.2f},{pitch.value:.2f},{yaw.value:.2f},{altitude.value:.2f}>>"
            ser.write((tx + '\n').encode())
            print(f'send -> {tx}')
            log_queue.put(('comm', f'Elapsed : {elapsed:.2f}, {tx}'))

            if ejection_flag.value:
                if deploy_time is None:
                    deploy_time = time.time()
                elif time.time() - deploy_time > 5:
                    break
            time.sleep(0.07)
    finally:
        ser.close()
        print(">> Communication thread terminated.")

# Sensor process
def sensor_process(roll, pitch, yaw, altitude, ejection_flag):
    i2c = busio.I2C(board.SCL, board.SDA)
    bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=0x76)
    bmp280.sea_level_pressure = 1013.25
    mpu = mpu6050(0x68)
    time.sleep(1.0)
    init_alt = bmp280.altitude

    # Stabilizing
    print("Stabilizing...")
    time.sleep(3)

    last_time = time.time()
    prev_alt = 0.0
    prev_roll = 0.0
    prev_pitch = 0.0
    prev_yaw = 0.0
    start_time = time.time()

    while not ejection_flag.value:
        now = time.time()
        dt = now - last_time
        last_time = now
        elapsed = now - start_time

        cali_alt = max(0.0, bmp280.altitude - init_alt)
        accel = mpu.get_accel_data()
        gyro = mpu.get_gyro_data()

        roll_angle = math.atan2(accel['y'], accel['z']) * 180 / math.pi
        pitch_angle = math.atan2(-accel['x'], math.sqrt(accel['y']**2 + accel['z']**2)) * 180 / math.pi
        yaw_angle = gyro['z']

        alpha = dt / (dt + (1 / (2 * math.pi * 20.0)))
        lpf_roll = alpha * roll_angle + (1 - alpha) * prev_roll
        lpf_pitch = alpha * pitch_angle + (1 - alpha) * prev_pitch
        lpf_yaw = alpha * yaw_angle + (1 - alpha) * prev_yaw
        lpf_alt = alpha * cali_alt + (1 - alpha) * prev_alt

        with roll_lock:
            roll.value = lpf_roll
            pitch.value = lpf_pitch
            yaw.value = lpf_yaw
        with alt_lock:
            altitude.value = lpf_alt

        # Logging
        log_queue.put(('gyro', f'Elapsed : {elapsed:.2f}, Roll : {lpf_roll:.2f}, Pitch : {lpf_pitch:.2f}, Yaw : {lpf_yaw:.2f}'))
        log_queue.put(('alt', f'Elapsed : {elapsed:.2f}, Alt : {lpf_alt:.2f}'))

        if abs(lpf_roll) > 45.0 or abs(lpf_pitch) > 45.0:
            print(f">> Angle Trigger: Roll={lpf_roll:.1f}, Pitch={lpf_pitch:.1f}")
            deploy_parachute()

        prev_roll = lpf_roll
        prev_pitch = lpf_pitch
        prev_yaw = lpf_yaw
        prev_alt = lpf_alt

        time.sleep(0.01)

    print(">> Sensor process terminated.")

# Main execution
if __name__ == "__main__":
    try:
        log_thread = threading.Thread(target=logging_thread)
        log_thread.start()

        comm = threading.Thread(target=comm_thread)
        comm.start()

        sensor_proc = multiprocessing.Process(target=sensor_process, args=(roll, pitch, yaw, altitude, ejection_flag))
        sensor_proc.start()

        comm.join()
        sensor_proc.join()

        log_queue.put(('EXIT', ''))
        log_thread.join()
    except KeyboardInterrupt:
        print(">> Interrupted by user")
    finally:
        GPIO.cleanup()
