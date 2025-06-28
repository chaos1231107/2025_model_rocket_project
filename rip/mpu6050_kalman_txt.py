import time
import math
from mpu6050 import mpu6050

mpu = mpu6050(0x68)
#kalman variables
kalman_angle = {'x' : 0.0, 'y' : 0.0}
bias = {'x' : 0.0, 'y' : 0.0}
P = {
    'x' : [[1.0, 0.0], [0.0, 1.0]],
    'y' : [[1.0, 0.0], [0.0, 1.0]]
}

Q_angle = 0.001
Q_bias = 0.003
R_measure = 0.03

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


lastTime = time.time()
try:
    while True:
        nowTime = time.time()
        dt = nowTime - lastTime
        lastTime = nowTime

        accel = mpu.get_accel_data()
        gyro = mpu.get_gyro_data()

        angle_roll = math.atan2(accel['y'], accel['z']) * 180 / math.pi
        angle_pitch = math.atan2(-accel['x'], math.sqrt(accel['y']**2 + accel['z']**2)) * 180 / math.pi

        gyro_x = gyro['x']
        gyro_y = gyro['y']

        kal_angle_roll = kalman_filter('x', angle_roll, gyro_x, dt)
        kal_angle_pitch = kalman_filter('y', angle_pitch, gyro_y, dt)

        print(f"Raw_Roll : {gyro_x:.2f}, Raw_Pitch : {gyro_y:.2f}, Kal_Roll : {kal_angle_roll:.2f}, Kal_Pitch : {kal_angle_pitch:.2f}")
        with open("MPU.txt", 'a') as f:
            f.write(f"{time.time():.2f}, {gyro_x:.2f}, {gyro_y:.2f}, {kal_angle_roll:.2f}, {kal_angle_pitch:.2f}\n")
        time.sleep(0.02)

except KeyboardIn
