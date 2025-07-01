import time
import board
import busio
import adafruit_bmp280

i2c = busio.I2C(board.SCL, board.SDA)
bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=0x76)

bmp280.sea_level_pressure = 1031.25
init_altitude = bmp280.altitude
cali_altitude = 0.0
try:

    while True:
        cali_altitude = bmp280.altitude - init_altitude
        print(f"altitude = {cali_altitude:.2f}")
        time.sleep(0.2)

except KeyboardInterrupt:
    print("00")
