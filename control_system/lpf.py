import numpy as np
import matplotlib.pyplot as plt

def sma_filter(data_list, window_size):
    sma_list = []
    for i in range(len(data_list)):
        if i < window_size - 1:
            sma_list.append(sum(data_list[0:i+1]) / (i+1))
        else:
            sma_list.append(sma_list[i-1] + (data_list[i] - data_list[i-window_size]) / window_size)
    return sma_list

def low_pass_filter(alpha, data_list): 
    lpf_list = []
    for i in range(len(data_list)):
        if i == 0:
            lpf_list.append(data_list[0])
        else:
            lpf_list.append(alpha * lpf_list[i-1] + (1 - alpha) * data_list[i])
    return lpf_list

alpha = 0.2
window_size = 3
data = np.random.randint(0, 100, size=100)
dt = 0.1
t = np.arange(len(data)) * dt

moving_avg = sma_filter(data, window_size)
lpf = low_pass_filter(alpha, data)

#print(len(data), len(moving_avg), len(lpf)) 
plt.plot(t, data, label='Raw Data')
plt.plot(t, moving_avg, label='SMA')
plt.plot(t, lpf, label='LPF')
plt.legend()
plt.show()
