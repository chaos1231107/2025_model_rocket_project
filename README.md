# 2025_model_rocket_project
# Developing Cascade Filter System to optimize the software of Rocket.

## SMA(Simple Moving Average) vs 1st Order Low Pass Filter(LPF)
- sma[k] = X[k] + (X[k] - X[k-window_size]) / window_size --> Using "slicing window" algorithm, same weight to all data, not sensitive to change of data.
- lpf[k] = (1-alpha) * sma[k-1] + alpha * data[k]

## Primary data processing : Reinforce stability of data
- (1-alhpa) * SMA[k-1] + alhpa*X[k] --> SMA + EMA belnding flilger = Low Pass Filter(LPF) : Eliminate high frequency sensor value : Only low frequency data can be passed

<img src="https://github.com/user-attachments/assets/0d932935-2bb8-4f4c-85a5-5843554de293" width="400"/>

## Secondary data Processing : Gain prediction by using Multivariate Kalman Filter(MKF) and correcting values by using PID control
- error = desired_value(state) - kalman_predict
