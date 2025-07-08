import time
import math
from mpu6050 import mpu6050
import RPi.GPIO as GPIO
import os
import board
import busio
import adafruit_bmp280

# Initialize I2C and sensors
i2c = busio.I2C(board.SCL, board.SDA)
bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=0x76)
bmp280.sea_level_pressure = 1013.25

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)

# MPU6050 Kalman filter variables
kalman_angle = {'x': 0.0, 'y': 0.0, 'z': 0.0}
bias = {'x': 0.0, 'y': 0.0, 'z': 0.0}
P = {
    'x': [[1.0, 0.0], [0.0, 1.0]],
    'y': [[1.0, 0.0], [0.0, 1.0]],
    'z': [[1.0, 0.0], [0.0, 1.0]]
}

Q_angle = 0.0025
Q_bias = 0.000025
R_measure = 0.25

# BMP280 variables
init_altitude = bmp280.altitude
cali_altitude = 0.0
prev_altitude = 0.0
lpf_cali_altitude = 0.0
prev_lpf_cali_altitude = 0.0
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

# MPU6050 variables
prev_kal_roll = 0.0
prev_kal_pitch = 0.0
prev_kal_yaw = 0.0
prev_lpf_roll_angle = 0.0
prev_lpf_pitch_angle = 0.0
prev_lpf_yaw_angle = 0.0
angle_yaw = 0.0
prev_lpf_gyro_roll = 0.0
prev_lpf_gyro_pitch = 0.0
prev_lpf_gyro_yaw = 0.0

# Deployment thresholds
threshold_roll_angle = 35.0
threshold_pitch_angle = 30.0
threshold_roll_rate = 200.0
threshold_pitch_rate = 250.0
trigger_hold_time = 0.1
trigger_timer = 0.0
ejection_flag = False

def kalman_filter(axis, new_angle, new_rate, dt):
    angle = kalman_angle[axis]
    b = bias[axis]
    p = P[axis]

    rate = new_rate - b
    angle += rate * dt
    p[0][0] += dt * (dt * p[1][1] - p[0][1] - p[1][0] + Q_angle)
    p[0][1] -= dt * p[1][1]
    p[1][0] -= dt * p[1][1]
    p[1][1] += Q_bias * dt

    S = p[0][0] + R_measure
    # Get kalman gain
    K = [p[0][0] / S, p[1][0] / S]
    y = new_angle - angle
    angle += K[0] * y
    b += K[1] * y

    p00 = p[0][0]
    p01 = p[0][1]
    p[0][0] -= K[0] * p00
    p[0][1] -= K[0] * p01
    p[1][0] -= K[1] * p00
    p[1][1] -= K[1] * p01

    kalman_angle[axis] = angle
    bias[axis] = b

    return angle

def stable_calibration(mpu, duration=3):
    print(f"Stabilizing MPU6050 Sensor for {duration}s")
    start = time.time()
    roll_list, pitch_list, yaw_list = [], [], []
    while time.time() - start < duration:
        GPIO.output(17, GPIO.LOW)
        accel = mpu.get_accel_data()
        gyro = mpu.get_gyro_data()

        roll = math.atan2(accel['y'], accel['z']) * 180 / math.pi
        pitch = math.atan2(-accel['x'], math.sqrt(accel['y']**2 + accel['z']**2))
        yaw = gyro['z']

        roll_list.append(roll)
        pitch_list.append(pitch)
        yaw_list.append(yaw)
        time.sleep(0.01)

    return (
        sum(roll_list) / len(roll_list),
        sum(pitch_list) / len(pitch_list),
        sum(yaw_list) / len(yaw_list)
    )

def deploy_parachute():
    """Deploy parachute by triggering GPIO pin"""
    global ejection_flag
    if not ejection_flag: 
        GPIO.output(17, GPIO.HIGH)
        print(">>Parachute Deployed")
        time.sleep(0.2)
        GPIO.output(17, GPIO.LOW)
        ejection_flag = True

# Create data files
data_files = ["combined_sensor_data.txt", "altitude_data.txt", "gyro_data.txt"]
for fname in data_files:
    if not os.path.exists(fname):
        with open(fname, 'w') as f:
            pass

# Initialize MPU6050
mpu = mpu6050(0x68)

# Get initial calibration values
init_roll, init_pitch, yaw_bias = stable_calibration(mpu)
kalman_angle['x'] = 0.0
kalman_angle['y'] = 0.0
kalman_angle['z'] = 0.0

print("System Stabilizing...3s")
time.sleep(3)

# Initialize timing variables
last_time = time.time()
start_time = time.time()

try:
    while True:
        now = time.time()
        dt = now - last_time
        elapsed = now - start_time
        last_time = now
        
        # Low-pass filter parameters
        fc = 24.22
        rc = 1.0 / (2 * math.pi * fc)
        alpha = dt / (rc + dt)
        
        # BMP280 altitude processing
        fc_bmp = 23.0
        rc_bmp = 1.0 / (2 * math.pi * fc_bmp)
        beta = dt / (rc_bmp + dt)
        
        # Read BMP280 data
        cali_altitude = bmp280.altitude - init_altitude
        if cali_altitude < 0.00:
            cali_altitude = 0.01
        
        # Kalman filter for altitude
        p_error += kalman_Q
        K2 = p_error / (p_error + kalman_R)
        kalman_estimate = prev_kalman_estimate + K2 * (cali_altitude - prev_kalman_estimate)
        p_error = (1.0 - K2) * prev_p_error
        
        # Low-pass filter for altitude
        lpf_cali_altitude = beta * kalman_estimate + (1.0 - beta) * prev_lpf_cali_altitude
        
        # Calculate falling velocity
        if abs(lpf_cali_altitude - prev_lpf_cali_altitude) > 0.01:
            falling_velocity = (lpf_cali_altitude - prev_lpf_cali_altitude) / dt
        
        # Check falling condition
        if falling_velocity <= -falling_velocity_threshold:
            falling_count += 1
       
        
        # Read MPU6050 data
        accel = mpu.get_accel_data()
        gyro = mpu.get_gyro_data()

        # Calculate angles
        angle_roll = math.atan2(accel['y'], accel['z']) * 180 / math.pi
        angle_pitch = math.atan2(-accel['x'], math.sqrt(accel['y']**2 + accel['z']**2))
        
        # Get relative angles
        relative_roll = angle_roll - init_roll
        relative_pitch = angle_pitch - init_pitch
        
        # Process gyroscope data
        gyro_x = gyro['x'] if abs(gyro['x']) <= 1000 else 0.0
        gyro_y = gyro['y'] if abs(gyro['y']) <= 1000 else 0.0
        gyro_z = gyro['z'] - yaw_bias
        gyro_z = gyro_z if abs(gyro_z) <= 1000 else 0.0
        
        # Integrate yaw axis
        angle_yaw += gyro_z * dt
        angle_yaw %= 360  # Normalization
        
        # Apply Kalman filter to angles
        kal_angle_roll = kalman_filter('x', relative_roll, gyro_x, dt)
        kal_angle_pitch = kalman_filter('y', relative_pitch, gyro_y, dt)
        kal_angle_yaw = kalman_filter('z', angle_yaw, gyro_z, dt)
        
        # Calculate angular velocities from Kalman filtered angles
        kal_gyro_roll = (kal_angle_roll - prev_kal_roll) / dt
        kal_gyro_pitch = (kal_angle_pitch - prev_kal_pitch) / dt
        kal_gyro_yaw = (kal_angle_yaw - prev_kal_yaw) / dt
        
        # Apply low-pass filter to angles and angular velocities
        lpf_roll_angle = alpha * kal_angle_roll + (1 - alpha) * prev_lpf_roll_angle
        lpf_pitch_angle = alpha * kal_angle_pitch + (1 - alpha) * prev_lpf_pitch_angle
        lpf_yaw_angle = alpha * kal_angle_yaw + (1 - alpha) * prev_lpf_yaw_angle
        
        lpf_gyro_roll = alpha * kal_gyro_roll + (1 - alpha) * prev_lpf_gyro_roll
        lpf_gyro_pitch = alpha * kal_gyro_pitch + (1 - alpha) * prev_lpf_gyro_pitch
        lpf_gyro_yaw = alpha * kal_gyro_yaw + (1 - alpha) * prev_lpf_gyro_yaw
        
        # Check deployment conditions
        # Condition 1: Orientation-based (tumbling)
        angle_cond = abs(lpf_roll_angle) > threshold_roll_angle or abs(lpf_pitch_angle) > threshold_pitch_angle
        rate_cond = abs(lpf_gyro_roll) > threshold_roll_rate or abs(lpf_gyro_pitch) > threshold_pitch_rate
        orientation_trigger = angle_cond or rate_cond
        
        # Condition 2: Altitude-based (falling)
        altitude_trigger = falling_count >= falling_threshold
        
        # Deploy if either condition is met
        if orientation_trigger:
            trigger_timer += dt
            if trigger_timer >= trigger_hold_time:
                deploy_parachute()
        elif altitude_trigger:
            deploy_parachute()
        else:
            trigger_timer = 0.0
        
        # Print status
        print(f"Time: {elapsed:.1f}s | "
              f"Alt: {lpf_cali_altitude:.2f}m | "
              f"Fall_V: {falling_velocity:.2f}m/s | "
              f"Fall_Count: {falling_count} | "
              f"Roll: {lpf_roll_angle:.1f}deg | "
              f"Pitch: {lpf_pitch_angle:.1f}deg | "
              f"Yaw : {lpf_yaw_angle :.1f}deg | "
              f"Deployed: {ejection_flag} | "
              )
        
        # Write combined data
        with open("combined_sensor_data.txt", 'a') as f:
            f.write(
                f"{elapsed:.3f}, {lpf_cali_altitude:.2f}, {falling_velocity:.2f}, {falling_count}, "
                f"{lpf_roll_angle:.2f}, {lpf_pitch_angle:.2f}, {lpf_yaw_angle:.2f}, "
                f"{lpf_gyro_roll:.2f}, {lpf_gyro_pitch:.2f}, {lpf_gyro_yaw:.2f}, "
                f"{int(ejection_flag)}\n"
            )
        
        # Write altitude data
        with open("altitude_data.txt", 'a') as f:
            f.write(
                f"{elapsed:.3f}, {cali_altitude:.2f}, {kalman_estimate:.2f}, {lpf_cali_altitude:.2f}\n"
            )
        
        # Write gyro data
        with open("gyro_data.txt", 'a') as f:
            f.write(
                f"{elapsed:.3f}, {gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f}, "
                f"{kal_gyro_roll:.2f}, {kal_gyro_pitch:.2f}, {kal_gyro_yaw:.2f}, "
                f"{lpf_gyro_roll:.2f}, {lpf_gyro_pitch:.2f}, {lpf_gyro_yaw:.2f}\n"
            )
        
        # Update previous values
        prev_kal_roll = kal_angle_roll
        prev_kal_pitch = kal_angle_pitch
        prev_kal_yaw = kal_angle_yaw
        
        prev_lpf_roll_angle = lpf_roll_angle
        prev_lpf_pitch_angle = lpf_pitch_angle
        prev_lpf_yaw_angle = lpf_yaw_angle
        
        prev_lpf_gyro_roll = lpf_gyro_roll
        prev_lpf_gyro_pitch = lpf_gyro_pitch
        prev_lpf_gyro_yaw = lpf_gyro_yaw
        
        prev_kalman_estimate = kalman_estimate
        prev_p_error = p_error
        prev_lpf_cali_altitude = lpf_cali_altitude
        
        time.sleep(0.002)

except KeyboardInterrupt:
    print("Program terminated by user")
    GPIO.cleanup()
except Exception as e:
    print(f"Error occurred: {e}")
    GPIO.cleanup()
finally:
    GPIO.cleanup()
