import time
import math
from mpu6050 import mpu6050
import os


#kalman variables
kalman_angle = {'x' : 0.0, 'y' : 0.0, 'z' : 0.0}
bias = {'x' : 0.0, 'y' : 0.0, 'z' : 0.0}
P = {
    'x' : [[1.0, 0.0], [0.0, 1.0]],
    'y' : [[1.0, 0.0], [0.0, 1.0]],
    'z' : [[1.0, 0.0], [0.0, 1.0]]
}

Q_angle = 0.0025
Q_bias = 0.000025
R_measure = 0.25

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
    #get kalman gain
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

#stabilize
def stable_calibration(mpu, duration=3):
    print(f"Stabilizing Sensor for {duration}s")
    start = time.time()
    roll_list, pitch_list, yaw_list = [], [], []
    while time.time() - start < duration:
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

#create file 
for fname in  ["expected_angle22.txt", "gyro_value4.txt"]:
	if not os.path.exists(fname):
		with open(fname, 'w') as f:
			pass
mpu = mpu6050(0x68)

#get init angle
init_roll, init_pitch, yaw_bias  = stable_calibration(mpu)
kalman_angle['x'] = 0.0
kalman_angle['y'] = 0.0
kalman_angle['z'] = 0.0

print("Sensor Stabilizing...3s")
time.sleep(3)

last_time = time.time()
start_time = time.time()
yaw_angel = 0.0
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
#weight of Low Pass Filter
#alpha = 0.606
try:
    while True:
        now = time.time()
        dt = now - last_time
        elapsed = now - start_time
        last_time = now
        
        fc = 24.22
        rc =  1.0 / (2*math.pi * fc)
        alpha = dt / (rc + dt)
        
        accel = mpu.get_accel_data()
        gyro = mpu.get_gyro_data()

        angle_roll = math.atan2(accel['y'], accel['z']) * 180 / math.pi 
        angle_pitch = math.atan2(-accel['x'], math.sqrt(accel['y']**2 + accel['z']**2))
        #get relative angle
        relative_roll = angle_roll - init_roll
        relative_pitch = angle_pitch - init_pitch
        #Angular Velocity
        gyro_x = gyro['x'] 
        if abs(gyro_x) > 1000:
            gyro_x = 0.0

        gyro_y = gyro['y'] 
        if abs(gyro_y) > 1000:
            gyro_y = 0.0
       
        gyro_z = (gyro['z'] - yaw_bias)
        if abs(gyro_z) > 1000:
            gyro_z = 0.0
        gyro_z = gyro_z - yaw_bias
        #Intergrate Yaw axis
        angle_yaw += gyro_z * dt
        angle_yaw %= 360 # normalization
        
        #kalman value
        kal_angle_roll = kalman_filter('x', relative_roll, gyro_x, dt)
        kal_angle_pitch = kalman_filter('y', relative_pitch, gyro_y, dt)
        kal_angle_yaw = kalman_filter('z', angle_yaw, gyro_z, dt)
        kal_gyro_roll = (kal_angle_roll - prev_kal_roll) / dt
        kal_gyro_pitch = (kal_angle_pitch - prev_kal_pitch) / dt
        kal_gyro_yaw = (kal_angle_yaw - prev_kal_yaw) / dt
        #lpf value
        lpf_roll_angle = alpha * kal_angle_roll + (1 - alpha) * prev_lpf_roll_angle
        lpf_pitch_angle = alpha * kal_angle_pitch + (1- alpha) * prev_lpf_pitch_angle
        lpf_yaw_angle = alpha * kal_angle_yaw + (1 - alpha) * prev_lpf_yaw_angle

        lpf_gyro_roll = alpha * kal_gyro_roll + (1- alpha) * prev_lpf_gyro_roll
        lpf_gyro_pitch = alpha * kal_gyro_pitch + (1 - alpha) * prev_lpf_gyro_pitch
        lpf_gyro_yaw = alpha * kal_gyro_yaw + (1 - alpha) * prev_lpf_gyro_yaw

        #print(f"Raw_Roll : {angle_roll:.2f}, Raw_Pitch : {angle_pitch:.2f}, Raw_Yaw : {angle_yaw:.2f}. Kal_Roll : {kal_angle_roll:.2f}, Kal_Pitch : {kal_angle_pitch:.2f} Kal_Yaw : {kal_angle_yaw:.2f}, LPF_Roll : {lpf_roll_angel:.2f}, LPF_Pitch : {lpf_pitch_angle:.2f"), LPF_Yaw : {lpf_yaw_angle:.2f}")
        #print(f"Raw_gyro_Roll : {gyro_x:.2f}, Raw_gyro_Pitch : {gyro_y:.2f}, Kal_velocity_Roll : {kal_gyro_roll:.2f}, Kal_velocity_Pitch : {kal_gyro_pitch:.2f}")
        #with open("MPU2.txt", 'a') as f:
            #f.write(f"{time.time():.2f}, {gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f}, {kal_angle_roll:.2f}, {kal_angle_pitch:.2f}, {kal_angle_yaw:.2f} ,{lpf_roll_angle:.2f}, {lpf_pitch_angle:.2f}, {lpf_yaw_angle:.2f}\n"
        
        print(
            f"Raw_Roll : {relative_roll:.2f}, Raw_Pitch : {relative_pitch:.2f}, Raw_Yaw : {angle_yaw:.2f}. "
            f"Kal_Roll : {kal_angle_roll:.2f}, Kal_Pitch : {kal_angle_pitch:.2f}, Kal_Yaw : {kal_angle_yaw:.2f}, "
            f"LPF_Roll : {lpf_roll_angle:.2f}, LPF_Pitch : {lpf_pitch_angle:.2f}, LPF_Yaw : {lpf_yaw_angle:.2f}"
            )

        with open("expected_angle22.txt", 'a') as f1:
            f1.write(
                f"{elapsed:.3f}, {angle_roll:.2f}, {angle_pitch:.2f}, {angle_yaw:.2f}, "
                f"{kal_angle_roll:.2f}, {kal_angle_pitch:.2f}, {kal_angle_yaw:.2f}, "
                f"{lpf_roll_angle:.2f}, {lpf_pitch_angle:.2f}, {lpf_yaw_angle:.2f}\n"
            )

        with open("gyro_value4.txt", 'a') as f2:
            f2.write(
                f"{elapsed:.3f}, {gyro_x:.2f}, {gyro_y:.2f}, {gyro_z:.2f}, "
                f"{kal_gyro_roll:.2f}, {kal_gyro_pitch:.2f}, {kal_gyro_yaw:.2f}, "
                f"{lpf_gyro_roll:.2f}, {lpf_gyro_pitch:.2f}, {lpf_gyro_yaw:.2f}\n"
            )

        prev_kal_roll = kal_angle_roll
        prev_kal_pitch = kal_angle_pitch
        prev_kal_yaw = kal_angle_yaw

        prev_lpf_roll_angle = lpf_roll_angle
        prev_lpf_pitch_angle = lpf_pitch_angle
        prev_lpf_yaw_angle = lpf_yaw_angle

        prev_lpf_gyro_roll = lpf_gyro_roll
        prev_lpf_gyro_pitch = lpf_gyro_pitch
        prev_lpf_gyro_yaw = lpf_gyro_yaw

        time.sleep(0.02)

except KeyboardInterrupt:
    print("00")
except Exception as e:
    print("file error")

