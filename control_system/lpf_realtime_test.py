import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def sma_filter(data_list, window_size):
    sma_list = []
    for i in range(len(data_list)):
        if i < window_size - 1:
            sma_list.append(sum(data_list[0:i+1]) / (i+1))
        else:
            sma_list.append(sma_list[i-1] + (data_list[i] - data_list[i-window_size]) / window_size)
    return sma_list

def ema_filter(sma_list, alpha, data_list):
    ema_list = []
    for i in range(len(data_list)):
        if i == 0:
            ema_list.append(sma_list[0])
        else:
            ema_list.append(alpha * sma_list[i-1] + (1 - alpha) * data_list[i])
    return ema_list

def low_pass_filter(data_list, beta, sma_list, ema_list):
    low_pass_list = []
    for i in range(len(data_list)):
        if i == 0:
            low_pass_list.append(ema_list[0])
        else:
            low_pass_list.append((1 - beta) * sma_list[i] + beta * ema_list[i])
    return low_pass_list


alpha = 0.2
beta = 0.125
window_size = 3
buffer_size = 100
dt = 0.1

data = []
time = []
filtered = []

fig, ax = plt.subplots()
raw_line, = ax.plot([], [], label='Raw Data')
filtered_line, = ax.plot([], [], label='LPF')
ax.set_xlim(0, buffer_size * dt)
ax.set_ylim(0, 100)
ax.set_title("Real-Time Filtered Data")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Value")
ax.legend()


def update(frame):
    new_data = np.random.randint(0, 100)  # get random data
    data.append(new_data)
    time.append(frame * dt)

    if len(data) > buffer_size:
        data.pop(0)
        time.pop(0)

    sma = sma_filter(data, window_size)
    ema = ema_filter(sma, alpha, data)
    lpf = low_pass_filter(data, beta, sma, ema)

    raw_line.set_data(time, data)
    filtered_line.set_data(time, lpf)

    ax.set_xlim(max(0, frame * dt - buffer_size * dt), frame * dt + dt)
    return raw_line, filtered_line

ani = FuncAnimation(fig, update, interval=100)
plt.tight_layout()
plt.grid(True)
plt.show()
