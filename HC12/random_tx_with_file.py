import serial
import time
import random
import string
import os

fname = 'rf_log.txt'
if not os.path.exists(fname):
    with open(fname, 'w') as f:
        pass
        
def random_string(n):
    pool = string.ascii_letters + string.digits
    return ''.join(random.choices(pool, k=n))

total_sent = 0
total_matched = 0

ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
#time.sleep(2)
print('Start pi Arduino Echo Test')

start_time = time.time()
last_time = time.time()
while True:
    now = time.time()
    dt = now - last_time
    elapsed = now - start_time
    last_time = now
    msg = random_string(128)
    ser.write((msg + '\n').encode())
    total_sent += 1
    print('TX:', msg)

    time.sleep(0.5)

    if ser.in_waiting:
        echo = ser.readline().decode(errors='ignore').strip()
        if echo == msg:
            total_matched += 1
            print(f'{elapsed:.2f}')
            print(f'RX : {msg}')
            print('matched')
        else:
            print('dismatched, RX:',repr(echo))
    else:
        print('No Signal')

    accuracy = (total_matched / total_sent) * 100
    with open(fname, 'a') as f:
        f.write(f'Elapsed Time : {elapsed:.2f}, TX DATA : {msg}, RX_DATA : {echo}, Accuracy : {accuracy:.2f}\n')
        
    print(f'Accuracy : {accuracy:.2f}% ')
    time.sleep(0.5)
