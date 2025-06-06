import matplotlib.pyplot as plt
import numpy as np

def moving_avg_filter(data_list, window_size):
    moving_avg_list = []
    if len(data_list) < window_size:
        return data_list
    else:
        initial_avg = sum(data_list[0:window_size]) / window_size
        moving_avg_list = [initial_avg] * (window_size -1) #padding -> initial_moving average : consider 2 loss data
        moving_avg_list.append(initial_avg)
       
        for i in range(window_size, len(data_list)):
            # Xk = Xk-1 + (Xk - Xk-window_size) / window_size (k > 1), 0-based indexing
            moving_avg_list.append(moving_avg_list[-1] + (data_list[i] - data_list[i-window_size]) / window_size) 
    return moving_avg_list

        
data_list = []
dt = 0.1 #sampling time
t = np.arange(0, 10, dt)

window_size = 3
for i in range(100):
    random_data = np.random.randint(0, 100)
    data_list.append(random_data)
    
moving_avg_gain = moving_avg_filter(data_list, window_size)

plt.plot(t, data_list, label='Raw Data')
plt.plot(t, moving_avg_gain, label='Moving Average Filter Output')
plt.legend()
plt.show()
    







