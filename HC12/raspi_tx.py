import serial
import time

ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
time.sleep(2)

print('Start echo relay test')

while True:
    ser.write(b'Ping from pi\n')
    print('Pi -> Arduino : Ping from Pi')
    time.sleep(0.5)

    if ser.in_waiting > 0:
        msg = ser.readline().decode('utf-8', errors='ignore').strip()
        print('Arduino -> Pi:', msg)

        relay = f'Rlay : {msg}\n'
        ser.write(relay.encode('utf-8'))
        print('Pi -> Arduino:', relay.strip())
    time.sleep(1)
