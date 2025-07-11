import serial
import time
import random
import string

def random_string(n=32):
    pool = string.ascii_letters + string.digits
    return ''.join(random.choices(pool, k=n))


ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
time.sleep(2)
print('Start pi Arduino Echo Test')

while True:
    msg = random_string(32)
    ser.write((msg + '\n').encode())
    print('TX:', msg)

    time.sleep(0.5)

    if ser.in_waiting:
        echo = ser.readline().decode(errors='ignore').strip()
        if echo == msg:
            print(f'RX : {msg}')
            print('matched')
        else:
            print('dismatched, RX:',repr(echo))
    else:
        print('No Signal')

    time.sleep(0.5)

