# 2025_model_rocket_project
## sma(simple moving average) vs ema(exponential moving average)
- sma[k] = X[k] + (X[k] - X[k-1]) / window_size
- ema[k] = (1-alpha) * sma[k-1] + alpha * data[k]
