import numpy as np
import matplotlib.pyplot as plt

def sma_initial(buffer, window):
    n = min(len(buffer), window)
    return sum(buffer) / n

def lowpass_filter(alpha, window):
    data_buffer = []
    lpf_buffer = []

    for i in range(100):
        new_data = np.random.randint(0, 100)
        data_buffer.append(new_data)

        if len(data_buffer) <= window:
            lpf_value = sma_initial(data_buffer, window)
        else:
            prev_lpf = lpf_buffer[-1]
            lpf_value = alpha * prev_lpf + (1 - alpha) * new_data

        lpf_buffer.append(lpf_value)

    return data_buffer, lpf_buffer

alpha = 0.2
window = 3
dt = 0.1
total_steps = 100

data_list, lpf_list = lowpass_filter(alpha, window)

plt.ion()
fig, ax = plt.subplots()
raw_line, = ax.plot([], [], label='Raw Data')
lpf_line, = ax.plot([], [], label='LPF')
ax.set_ylim(0, 100)
ax.set_xlim(0, total_steps * dt)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Value')
ax.set_title('Low-Pass Filter processing')
ax.legend()

t = []

for i in range(total_steps):
    t.append(i * dt)

    raw_line.set_data(t[:i+1], data_list[:i+1])
    lpf_line.set_data(t[:i+1], lpf_list[:i+1])
    ax.set_xlim(max(0, t[i] - 10), t[i] + dt)
    plt.pause(dt)

plt.ioff()
plt.show()
