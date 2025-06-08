import numpy as np
import matplotlib.pyplot as plt

def low_pass_filter(alpha, data_list, window_size): 
    lpf_list = []
    for i in range(len(data_list)):
        if i < window_size:
            lpf_list.append(data_list[i])
        else:
            lpf_list.append((1-alpha) * lpf_list[i-1] + alpha * data_list[i])
    return lpf_list

def Kalman_filter(z_list, Q, R):
    x_est_list = []
    P = 25 #initial prediction error covariance
    x_est = z_list[0]
    
    for z in z_list:
        
        x_pred = x_est
        p_pred = P + Q
        K = p_pred / (p_pred + R)
        x_est = x_pred + K * (z - x_pred)
        P = (1 - K) * p_pred
        x_est_list.append(x_est)
    return x_est_list

alpha = 0.3
window_size = 3
Q = 0.1 # noise of preiction
R = 1 #noise of measured value : 

data = np.random.randint(0, 100, size=100)
dt = 0.1
t = np.arange(len(data)) * dt

#moving_avg = sma_filter(data, window_size)
lpf = low_pass_filter(alpha, data, window_size)
kmf = Kalman_filter(lpf, Q, R)

#print(len(data), len(moving_avg), len(lpf)) 
plt.plot(t, data, label='Raw Data')
#plt.plot(t, moving_avg, label='SMA')
plt.plot(t, lpf, label='LPF')
plt.plot(t, kmf, label='KF')
plt.legend()
plt.show()
