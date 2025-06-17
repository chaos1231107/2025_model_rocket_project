import matplotlib.pyplot as plt
import numpy as np

dt = 0.1
t = [0]
for i in range(10):
	t.append(t[-1] + dt)

output = [0, 0, 5, 12, 18, 22, 24, 25, 25.5, 26, 26]
t_array = np.array(t)
output_array = np.array(output)

slope = np.diff(output_array) / np.diff(t_array)
max_slope_idx = np.argmax(slope)
max_slope = slope[max_slope_idx]

x0 = t_array[max_slope_idx]
y0 = output_array[max_slope_idx]

tangent_line = max_slope * (t_array  - x0) + y0 

plt.plot(t, output, marker='o')
plt.plot(t_array, tangent_line, 'r--', label = "Inflection Tangent")
plt.xlabel("Time (s)")
plt.ylabel("Step Output")
plt.grid(True)
plt.legend()
plt.show()


