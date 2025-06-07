# 2025_model_rocket_project
# Developing Cascade Filter System to optimize the software of Rocket.

## SMA(Simple Moving Average) vs EMA(exponential moving average)
- sma[k] = X[k] + (X[k] - X[k-window]) / window_size --> Using "slicing window" algorithm
- ema[k] = (1-alpha) * sma[k-1] + alpha * data[k]

## Primary data Processing : reinforce stability of data
- (1-beta) * SMA_value + beta * EMA_value --> SMA + EMA belnding flilger = Low Pass Filter(LPF) : Eliminate high frequency sensor value : Only low frequency data can be passed
- ![image](https://github.com/user-attachments/assets/0d932935-2bb8-4f4c-85a5-5843554de293)

## Secondary data Processing : Gain prediction by using Multivariate Kalman Filter(MKF) and correcting values by using PID control
- error = desired_value(state) - kalman_predict
