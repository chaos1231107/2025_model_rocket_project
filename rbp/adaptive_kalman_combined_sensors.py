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

# Enhanced MPU6050 Kalman filter variables with adaptive parameters
kalman_angle = {'x': 0.0, 'y': 0.0, 'z': 0.0}
bias = {'x': 0.0, 'y': 0.0, 'z': 0.0}
P = {
    'x': [[1.0, 0.0], [0.0, 1.0]],
    'y': [[1.0, 0.0], [0.0, 1.0]],
    'z': [[1.0, 0.0], [0.0, 1.0]]
}

# Adaptive noise parameters
Q_angle_base = 0.001  # Reduced base process noise for faster response
Q_bias_base = 0.000001
R_measure_base = 0.1  # Reduced measurement noise for faster response

# Dynamic tuning parameters
motion_threshold = 50.0  # deg/s threshold for motion detection
high_motion_multiplier = 10.0  # Multiplier for high motion scenarios
acceleration_threshold = 2.0  # g threshold for acceleration detection

# BMP280 enhanced variables
init_altitude = bmp280.altitude
cali_altitude = 0.0
prev_altitude = 0.0
lpf_cali_altitude = 0.0
prev_lpf_cali_altitude = 0.0
p_error = 0.0
kalman_Q_alt = 0.01  # Reduced for faster response
kalman_estimate = 0.0
prev_kalman_estimate = 0.0
prev_p_error = 0.0
kalman_R_alt = 0.5  # Reduced for faster response
falling_count = 0
falling_threshold = 2  # Reduced threshold for faster detection
falling_velocity = 0.0
falling_velocity_threshold = 0.15  # Reduced threshold

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

# Motion detection variables
motion_detected = False
high_acceleration = False
prev_accel_magnitude = 0.0

# Enhanced deployment thresholds
threshold_roll_angle = 30.0  # Reduced for faster detection
threshold_pitch_angle = 25.0  # Reduced for faster detection
threshold_roll_rate = 150.0  # Reduced for faster detection
threshold_pitch_rate = 180.0  # Reduced for faster detection
trigger_hold_time = 0.05  # Reduced hold time for faster response
trigger_timer = 0.0
ejection_flag = False

def adaptive_kalman_filter(axis, new_angle, new_rate, dt, motion_level=1.0):
    """Enhanced Kalman filter with adaptive parameters"""
    angle = kalman_angle[axis]
    b = bias[axis]
    p = P[axis]

    # Adaptive noise parameters based on motion level
    Q_angle = Q_angle_base * motion_level
    Q_bias = Q_bias_base * motion_level
    R_measure = R_measure_base / motion_level  # Lower R for higher motion

    rate = new_rate - b
    angle += rate * dt
    
    # Prediction step with adaptive Q
    p[0][0] += dt * (dt * p[1][1] - p[0][1] - p[1][0] + Q_angle)
    p[0][1] -= dt * p[1][1]
    p[1][0] -= dt * p[1][1]
    p[1][1] += Q_bias * dt

    # Update step with adaptive R
    S = p[0][0] + R_measure
    K = [p[0][0] / S, p[1][0] / S]
    y = new_angle - angle
    angle += K[0] * y
    b += K[1] * y

    # Covariance update
    p00 = p[0][0]
    p01 = p[0][1]
    p[0][0] -= K[0] * p00
    p[0][1] -= K[0] * p01
    p[1][0] -= K[1] * p00
    p[1][1] -= K[1] * p01

    kalman_angle[axis] = angle
    bias[axis] = b

    return angle

def detect_motion_level(gyro_data, accel_data):
    """Detect motion level for adaptive filtering"""
    global motion_detected, high_acceleration, prev_accel_magnitude
    
    # Calculate gyroscope magnitude
    gyro_magnitude = math.sqrt(gyro_data['x']**2 + gyro_data['y']**2 + gyro_data['z']**2)
    
    # Calculate acceleration magnitude
    accel_magnitude = math.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)
    accel_change = abs(accel_magnitude - prev_accel_magnitude)
    prev_accel_magnitude = accel_magnitude
    
    # Detect motion states
    motion_detected = gyro_magnitude > motion_threshold
    high_acceleration = accel_change > acceleration_threshold
    
    # Calculate motion level multiplier
    motion_level = 1.0
    if motion_detected:
        motion_level *= 3.0
    if high_acceleration:
        motion_level *= 2.0
    if gyro_magnitude > motion_threshold * 2:
        motion_level *= high_motion_multiplier
    
    return min(motion_level, 20.0)  # Cap the multiplier

def stable_calibration(mpu, duration=3):
    """Enhanced calibration with outlier rejection"""
    print(f"Stabilizing MPU6050 Sensor for {duration}s")
    start = time.time()
    roll_list, pitch_list, yaw_list = [], [], []
    
    while time.time() - start < duration:
        GPIO.output(17, GPIO.LOW)
        accel = mpu.get_accel_data()
        gyro = mpu.get_gyro_data()

        # Calculate angles
        roll = math.atan2(accel['y'], accel['z']) * 180 / math.pi
        pitch = math.atan2(-accel['x'], math.sqrt(accel['y']**2 + accel['z']**2)) * 180 / math.pi
        yaw = gyro['z']

        # Simple outlier rejection
        if len(roll_list) > 10:
            roll_mean = sum(roll_list[-10:]) / 10
            pitch_mean = sum(pitch_list[-10:]) / 10
            yaw_mean = sum(yaw_list[-10:]) / 10
            
            if (abs(roll - roll_mean) < 10 and 
                abs(pitch - pitch_mean) < 10 and 
                abs(yaw - yaw_mean) < 50):
                roll_list.append(roll)
                pitch_list.append(pitch)
                yaw_list.append(yaw)
        else:
            roll_list.append(roll)
            pitch_list.append(pitch)
            yaw_list.append(yaw)
        
        time.sleep(0.01)

    # Remove outliers using median-based approach
    roll_list.sort()
    pitch_list.sort()
    yaw_list.sort()
    
    n = len(roll_list)
    return (
        sum(roll_list[n//4:3*n//4]) / (n//2),  # Use middle 50% for robustness
        sum(pitch_list[n//4:3*n//4]) / (n//2),
        sum(yaw_list[n//4:3*n//4]) / (n//2)
    )

def deploy_parachute():
    """Enhanced deployment with confirmation"""
    global ejection_flag
    if not ejection_flag: 
        GPIO.output(17, GPIO.HIGH)
        print(">>PARACHUTE DEPLOYED<<")
        time.sleep(0.2)
        GPIO.output(17, GPIO.LOW)
        ejection_flag = True
        
        # Log deployment
        with open("deployment_log.txt", 'a') as f:
            f.write(f"Deployment at {time.time():.3f}\n")

def emergency_deployment_check(roll, pitch, roll_rate, pitch_rate):
    """Emergency deployment for extreme conditions"""
    extreme_angle = abs(roll) > 60 or abs(pitch) > 60
    extreme_rate = abs(roll_rate) > 400 or abs(pitch_rate) > 400
    
    if extreme_angle or extreme_rate:
        print(f"EMERGENCY CONDITION: Roll={roll:.1f}, Pitch={pitch:.1f}, "
              f"Roll_rate={roll_rate:.1f}, Pitch_rate={pitch_rate:.1f}")
        return True
    return False

# Create enhanced data files
data_files = ["combined_sensor_data.txt", "altitude_data.txt", "gyro_data.txt", "deployment_log.txt"]
for fname in data_files:
    if not os.path.exists(fname):
        with open(fname, 'w') as f:
            if fname == "combined_sensor_data.txt":
                f.write("Time,Altitude,Fall_Velocity,Fall_Count,Roll,Pitch,Yaw,"
                       "Roll_Rate,Pitch_Rate,Yaw_Rate,Motion_Level,Deployed\n")
            elif fname == "altitude_data.txt":
                f.write("Time,Raw_Altitude,Kalman_Altitude,Filtered_Altitude\n")
            elif fname == "gyro_data.txt":
                f.write("Time,Raw_Gyro_X,Raw_Gyro_Y,Raw_Gyro_Z,"
                       "Kal_Rate_X,Kal_Rate_Y,Kal_Rate_Z,"
                       "Filtered_Rate_X,Filtered_Rate_Y,Filtered_Rate_Z\n")

# Initialize MPU6050
mpu = mpu6050(0x68)

# Get initial calibration values
print("Starting enhanced calibration...")
init_roll, init_pitch, yaw_bias = stable_calibration(mpu)
print(f"Calibration complete: Roll={init_roll:.2f}, Pitch={init_pitch:.2f}, Yaw_bias={yaw_bias:.2f}")

# Reset Kalman filters
kalman_angle = {'x': 0.0, 'y': 0.0, 'z': 0.0}
bias = {'x': 0.0, 'y': 0.0, 'z': 0.0}

print("System Ready - Enhanced Responsive Mode")
time.sleep(2)

# Initialize timing variables
last_time = time.time()
start_time = time.time()

try:
    while True:
        now = time.time()
        dt = now - last_time
        elapsed = now - start_time
        last_time = now
        
        # Read sensor data
        accel = mpu.get_accel_data()
        gyro = mpu.get_gyro_data()
        
        # Detect motion level for adaptive filtering
        motion_level = detect_motion_level(gyro, accel)
        
        # Dynamic low-pass filter parameters based on motion
        base_fc = 35.0 if motion_level > 3.0 else 20.0  # Higher cutoff for high motion
        fc = base_fc * min(motion_level, 3.0)
        rc = 1.0 / (2 * math.pi * fc)
        alpha = dt / (rc + dt)
        
        # BMP280 altitude processing with adaptive filtering
        fc_bmp = 30.0 if motion_level > 2.0 else 20.0
        rc_bmp = 1.0 / (2 * math.pi * fc_bmp)
        beta = dt / (rc_bmp + dt)
        
        # Read and process BMP280 data
        cali_altitude = bmp280.altitude - init_altitude
        if cali_altitude < 0.00:
            cali_altitude = 0.01
        
        # Enhanced Kalman filter for altitude with adaptive parameters
        kalman_Q_adaptive = kalman_Q_alt * (motion_level if motion_level > 1.0 else 1.0)
        kalman_R_adaptive = kalman_R_alt / (motion_level if motion_level > 1.0 else 1.0)
        
        p_error += kalman_Q_adaptive
        K2 = p_error / (p_error + kalman_R_adaptive)
        kalman_estimate = prev_kalman_estimate + K2 * (cali_altitude - prev_kalman_estimate)
        p_error = (1.0 - K2) * prev_p_error
        
        # Enhanced low-pass filter for altitude
        lpf_cali_altitude = beta * kalman_estimate + (1.0 - beta) * prev_lpf_cali_altitude
        
        # Calculate falling velocity with smoothing
        if abs(lpf_cali_altitude - prev_lpf_cali_altitude) > 0.005:
            falling_velocity = (lpf_cali_altitude - prev_lpf_cali_altitude) / dt
        
        # Enhanced falling detection
        if falling_velocity <= -falling_velocity_threshold:
            falling_count += 1
        else:
            falling_count = max(0, falling_count - 1)  # Gradual decay
        
        # Calculate angles
        angle_roll = math.atan2(accel['y'], accel['z']) * 180 / math.pi
        angle_pitch = math.atan2(-accel['x'], math.sqrt(accel['y']**2 + accel['z']**2)) * 180 / math.pi
        
        # Get relative angles
        relative_roll = angle_roll - init_roll
        relative_pitch = angle_pitch - init_pitch
        
        # Enhanced gyroscope processing with adaptive limits
        gyro_limit = 1500 if motion_level > 5.0 else 1000
        gyro_x = gyro['x'] if abs(gyro['x']) <= gyro_limit else 0.0
        gyro_y = gyro['y'] if abs(gyro['y']) <= gyro_limit else 0.0
        gyro_z = gyro['z'] - yaw_bias
        gyro_z = gyro_z if abs(gyro_z) <= gyro_limit else 0.0
        
        # Integrate yaw axis
        angle_yaw += gyro_z * dt
        angle_yaw = angle_yaw % 360
        
        # Apply adaptive Kalman filter to angles
        kal_angle_roll = adaptive_kalman_filter('x', relative_roll, gyro_x, dt, motion_level)
        kal_angle_pitch = adaptive_kalman_filter('y', relative_pitch, gyro_y, dt, motion_level)
        kal_angle_yaw = adaptive_kalman_filter('z', angle_yaw, gyro_z, dt, motion_level)
        
        # Calculate angular velocities from Kalman filtered angles
        kal_gyro_roll = (kal_angle_roll - prev_kal_roll) / dt if dt > 0 else 0.0
        kal_gyro_pitch = (kal_angle_pitch - prev_kal_pitch) / dt if dt > 0 else 0.0
        kal_gyro_yaw = (kal_angle_yaw - prev_kal_yaw) / dt if dt > 0 else 0.0
        
        # Apply adaptive low-pass filter to angles and angular velocities
        lpf_roll_angle = alpha * kal_angle_roll + (1 - alpha) * prev_lpf_roll_angle
        lpf_pitch_angle = alpha * kal_angle_pitch + (1 - alpha) * prev_lpf_pitch_angle
        lpf_yaw_angle = alpha * kal_angle_yaw + (1 - alpha) * prev_lpf_yaw_angle
        
        lpf_gyro_roll = alpha * kal_gyro_roll + (1 - alpha) * prev_lpf_gyro_roll
        lpf_gyro_pitch = alpha * kal_gyro_pitch + (1 - alpha) * prev_lpf_gyro_pitch
        lpf_gyro_yaw = alpha * kal_gyro_yaw + (1 - alpha) * prev_lpf_gyro_yaw
        
        # Enhanced deployment conditions
        # Condition 1: Orientation-based (tumbling) with adaptive thresholds
        dynamic_angle_threshold = threshold_roll_angle * (0.8 if motion_level > 5.0 else 1.0)
        dynamic_rate_threshold = threshold_roll_rate * (0.8 if motion_level > 5.0 else 1.0)
        
        angle_cond = (abs(lpf_roll_angle) > dynamic_angle_threshold or 
                     abs(lpf_pitch_angle) > threshold_pitch_angle)
        rate_cond = (abs(lpf_gyro_roll) > dynamic_rate_threshold or 
                    abs(lpf_gyro_pitch) > threshold_pitch_rate)
        
        orientation_trigger = angle_cond or rate_cond
        
        # Condition 2: Altitude-based (falling)
        altitude_trigger = falling_count >= falling_threshold
        
        # Condition 3: Emergency deployment
        emergency_trigger = emergency_deployment_check(lpf_roll_angle, lpf_pitch_angle, 
                                                      lpf_gyro_roll, lpf_gyro_pitch)
        
        # Deploy parachute logic
        if emergency_trigger:
            deploy_parachute()
        elif orientation_trigger:
            trigger_timer += dt
            if trigger_timer >= trigger_hold_time:
                deploy_parachute()
        elif altitude_trigger:
            deploy_parachute()
        else:
            trigger_timer = 0.0
        
        # Enhanced status display
        motion_status = "HIGH" if motion_level > 5.0 else "MED" if motion_level > 2.0 else "LOW"
        print(f"T:{elapsed:.1f}s | Alt:{lpf_cali_altitude:.2f}m | FV:{falling_velocity:.2f} | "
              f"FC:{falling_count} | R:{lpf_roll_angle:.1f}Â° | P:{lpf_pitch_angle:.1f}Â° | "
              f"Y:{lpf_yaw_angle:.1f}Â° | RR:{lpf_gyro_roll:.0f}Â°/s | PR:{lpf_gyro_pitch:.0f}Â°/s | "
              f"Motion:{motion_status} | ML:{motion_level:.1f} | Deployed:{ejection_flag}")
        
        # Write enhanced combined data
        with open("combined_sensor_data.txt", 'a') as f:
            f.write(f"{elapsed:.3f},{lpf_cali_altitude:.2f},{falling_velocity:.2f},{falling_count},"
                   f"{lpf_roll_angle:.2f},{lpf_pitch_angle:.2f},{lpf_yaw_angle:.2f},"
                   f"{lpf_gyro_roll:.2f},{lpf_gyro_pitch:.2f},{lpf_gyro_yaw:.2f},"
                   f"{motion_level:.2f},{int(ejection_flag)}\n")
        
        # Write altitude data
        with open("altitude_data.txt", 'a') as f:
            f.write(f"{elapsed:.3f},{cali_altitude:.2f},{kalman_estimate:.2f},{lpf_cali_altitude:.2f}\n")
        
        # Write gyro data
        with open("gyro_data.txt", 'a') as f:
            f.write(f"{elapsed:.3f},{gyro_x:.2f},{gyro_y:.2f},{gyro_z:.2f},"
                   f"{kal_gyro_roll:.2f},{kal_gyro_pitch:.2f},{kal_gyro_yaw:.2f},"
                   f"{lpf_gyro_roll:.2f},{lpf_gyro_pitch:.2f},{lpf_gyro_yaw:.2f}\n")
        
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
        
        time.sleep(0.001)  # Reduced sleep time for faster response

except KeyboardInterrupt:
    print("\nProgram terminated by user")
    GPIO.cleanup()
except Exception as e:
    print(f"Error occurred: {e}")
    GPIO.cleanup()
finally:
    GPIO.cleanup()
    print("GPIO cleanup completed")
