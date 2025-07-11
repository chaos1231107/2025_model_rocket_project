import serial
import time
import random
import string

def random_string(n=100):
    pool = string.ascii_letters + string.digits
    return ''.join(random.choices(pool, k=n))

total_sent = 0
total_matched = 0

ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
time.sleep(2)
print('Start pi Arduino Echo Test')

while True:
    msg = random_string(64)
    ser.write((msg + '\n').encode())
    total_sent += 1
    print('TX:', msg)

    time.sleep(0.5)

    if ser.in_waiting:
        echo = ser.readline().decode(errors='ignore').strip()
        if echo == msg:
            total_matched += 1
            print(f'RX : {msg}')
            print('matched')
        else:
            print('dismatched, RX:',repr(echo))
    else:
        print('No Signal')

    accuracy = (total_matched / total_sent) * 100
    print(f'Accuracy : {accuracy:.2f}% ')
    time.sleep(0.5)
