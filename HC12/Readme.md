## Circuit configuration for Raspberry Pi and Arduino communication test

### HC12 RF Module - Raspberry Pi

#### 5V - VCC
#### GND - GND
#### RXD - GPIO 14(TXD)
#### TXD - GPIO 15(RXD), Require 4kohm Register connected in series, Prevent overcurrent at TXD PIN to RBP

### HC12 RF Module - Arduino

#### 5V - VCC
#### GND - GND
#### RXD - D3
#### TXD - D2

## Library installation (python3-serial)
### sudo apt install python3-serial

## Raspberry Pi UART port confirmation 
### Step 0 : sudo raspi-config -> login shell : NO, Serial enable : Yes 
### Step 1 : sudo nano /boot/firmware/config.txt
### Step 2 : sudo nano /boot/firmware/config.txt
- **Access bash file and edit** 
- **enable_uart=1**
- **dtverlay=disable-bt**
- **dtverlay=uart0, txd0_pin=14, rxd0_pin=15**
<img width="1731" height="715" alt="image" src="https://github.com/user-attachments/assets/868f29bc-7ea1-4f30-bca8-6801c9fe0208" />

### Step 3 : Deactivate serial-getty service
- **sudo systemctl disable --now serial-getty@ttyAMA0.service**
### Step 4 : Reboot
- **sudo reboot**
### Get UART PORT 
- **ls -l /dev/serial* -> /dev/serial0 -> tty/AMA0 : Adress of UART PIN(UART needs root authorization to be executed)
<img width="1003" height="154" alt="image" src="https://github.com/user-attachments/assets/1bd71b40-2864-48f7-bef3-03352aaaeffb" />


<img width="342" height="82" alt="image" src="https://github.com/user-attachments/assets/e00a954b-5bde-46c0-93d6-b4680745553d" />

## RF Signal-Based Communication with RBP(TX) and Arduino(RX)

https://github.com/user-attachments/assets/bb00b3bc-3c95-4126-8bbb-460e6d47139b

## References

https://www.instructables.com/Radio-Telemetry-for-a-Model-Rocket/

