import numpy as np
import matplotlib.pyplot as plt
import time

def sma_initial(buffer, window):
    n = min(len(buffer), window)
    return sum(buffer) / n

def low_pass_filter(alpha=0.3, window=3, dt=0.1, total_steps=100):
    data_buffer = []
    lpf_buffer = []

    plt.ion()
    fig, ax = plt.subplots()
    raw_line, = ax.plot([], [], label='Raw Data')
    lpf_line, = ax.plot([], [], label='LPF')
    ax.set_ylim(0, 100)
    ax.set_xlim(0, total_steps * dt)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Value')
    ax.set_title('Real-Time LPF Processing')
    ax.legend()

    t = []

    for i in range(total_steps):
       
        new_data = np.random.randint(0, 100)
        data_buffer.append(new_data)
        t.append(i * dt)

        if len(data_buffer) <= window:
            lpf_value = sma_initial(data_buffer, window)
        else:
            prev_lpf = lpf_buffer[-1]
            lpf_value = (1-alpha) * prev_lpf + alpha * new_data

        lpf_buffer.append(lpf_value)

        raw_line.set_data(t, data_buffer)
        lpf_line.set_data(t, lpf_buffer)
        ax.set_xlim(max(0, t[i] - 10), t[i] + dt)

        plt.pause(dt)
        time.sleep(dt)  

    plt.ioff()
    plt.show()

low_pass_filter()
