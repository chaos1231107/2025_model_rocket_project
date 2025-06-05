import numpy as np
import matplotlib.pyplot as plt

# 초기 상태
theta = [0.5]        # 초기 각도 (rad)
omega = 0.0          # 초기 각속도
dt = 0.01            # 시간 간격 (s)
g = 9.81             # 중력가속도 (m/s^2)
L = 1.0              # 진자의 길이 (m)

# PID 제어 계수
Kp = 30.0
Ki = 1.0
Kd = 5.0
integral = 0.0
prev_error = 0.0

# 로그 저장
theta_log = []
time_log = []

# 시뮬레이션 반복
for i in range(1000):
    t = i * dt
    time_log.append(t)

    # 가상 센서값 (노이즈 포함)
    imu_angle = theta[-1] + np.random.normal(0, 0.01)
    imu_angular_velocity = omega + np.random.normal(0, 0.01)

    # PID 계산
    error = 0.0 - imu_angle  # 목
    integral += error * dt
    derivative = (error - prev_error) / dt
    prev_error = error
    torque = Kp * error + Ki * integral + Kd * derivative

    # 운동 방정식 (진자에 torque 적용)
    alpha = -g/L * np.sin(theta[-1]) + torque
    omega += alpha * dt
    new_theta = theta[-1] + omega * dt
    theta.append(new_theta)
    theta_log.append(new_theta)

# 플롯 (time_log, theta_log 길이 맞춤)
plt.plot(time_log, theta_log)
plt.title("Simulated IMU Angle with PID Control")
plt.xlabel("Time (s)")
plt.ylabel("Angle (rad)")
plt.grid()
plt.show()
