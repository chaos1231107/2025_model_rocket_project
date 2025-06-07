import numpy as np
import matplotlib.pyplot as plt

def sma_filter(data_list, window_size):
    sma_list = []
    for i in range(len(data_list)):
        if i < window_size - 1:
            sma_list.append(sum(data_list[0:i+1]) / (i+1))
        else:
            # SMA[k] = SMA[k-1] + (X_data[k] - X_data[k-window_size]) / window_size
            sma_list.append(sma_list[-1] + (data_list[i] - data_list[i-window_size]) / window_size)
    return sma_list

def ema_filter(sma_list, alpha, data_list):
    ema_list = []
    for i in range(len(data_list)):
        if i == 0:
            ema_list.append(sma_list[0])
        else:
            ema_list.append(alpha * sma_list[-1] + (1 - alpha) * data_list[i]) #EMA = alpha * SMA[i-1] + (1-alpha) * X_data[i]
    return ema_list

def low_pass_filter(data_list, beta, sma_list, ema_list):
    low_pass_list = []
    for i in range(len(data_list)):
        if i == 0:
            low_pass_list.append(ema_list[0])
        else:
            low_pass_list.append((1-beta) * sma_list[i] + beta * ema_list[i]) #LPF = (1-beta) * SMA + beta * EMA
    return low_pass_list

alpha = 0.2 #weight of previous data : give more weight to previous data
beta = 0.125
window_size = 3
data = np.random.randint(0, 100, size=100)
dt = 0.1
t = np.arange(0, 10, dt) 
moving_avg = sma_filter(data, window_size)
ema_avg = ema_filter(moving_avg, alpha, data)
lpf = low_pass_filter(data, beta, moving_avg, ema_avg)

plt.plot(t, data, label='Raw Data')
plt.plot(t, moving_avg, label='SMA')
plt.plot(t, ema_avg, label='EMA')
plt.plot(t, lpf, label='LPF')
plt.legend()
plt.show()

