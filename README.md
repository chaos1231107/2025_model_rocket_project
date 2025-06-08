# 2025_model_rocket_project
# Primary data processing 
## Developing Filter System to optimize the software of Rocket.

## SMA(Simple Moving Average) vs 1st Order Low Pass Filter(LPF)
- sma[k] = X[k] + (X[k] - X[k-window_size]) / window_size --> Using "slicing window" algorithm, same weight to all data, not sensitive to change of data.
- lpf[k] = (1-alpha) * lpf[k-1] + alpha * data[k]

## Raw data vs SMA(Simple Moving Average) vs LPF(Low Pass Filter)
- SMA(Simple Moving Average) : Eliminate Noise but Not sensitive to sudden change, give same weight to all data
- LPF(Low Pass Filter) : Eliminate high frequency signal & sensitive to sudden change, give less weight to previous data
<img src="https://github.com/user-attachments/assets/a8ebf96d-3277-48e0-92ab-1b994544b1d0" width="400" />

## Secondary data processing : Gain prediction by using Multivariate Kalman Filter(MKF) and correcting values by using PID control
- error = desired_value(state) - kalman_predict
