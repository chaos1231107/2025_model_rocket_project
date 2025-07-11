## Circuit configuration for Raspberry Pi and Arduino communication test

### HC12 RF Module - Raspberry Pi

#### 5V - VCC
#### GND - GND
#### RXD - GPIO 14(TXD)
#### TXD - GPIO 15(RXD)

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
#### Access bash file 
**enable_uart=1**
**dtverlay=disable-bt**
**dtverlay=uart0, txd0_pin=14, rxd0_pin=15**

<img width="342" height="82" alt="image" src="https://github.com/user-attachments/assets/e00a954b-5bde-46c0-93d6-b4680745553d" />

## RF Signal-Based Communication with RBP(TX) and Arduino(RX)

https://github.com/user-attachments/assets/bb00b3bc-3c95-4126-8bbb-460e6d47139b

