# 2025_model_rocket_project
# Developing Cascade Filter System to optimize the software of Rocket.

## SMA(Simple Moving Average) vs EMA(exponential moving average)
- sma[k] = X[k] + (X[k] - X[k-1]) / window_size --> Using "slicing window" algorithm
- ema[k] = (1-alpha) * sma[k-1] + alpha * data[k]

## Primary data Processing : reinforce stability of data
- SMA + EMA belnding flilger = Primary Low Pass Filter(LPF) : Eliminate high frequency sensor value : Only low frequency data can be passed

## Secondary data Processing : Gain prediction by using Multivariate Kalman Filter(MKF) and correcting values by using PID control
- error = desired_value(state) - kalman_predict
