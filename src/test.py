import serial
import time

CHIP_ID_ADDR = 0x00
COM_START_BYTE_WR = 0xAA
COM_READ = 0x01
COM_START_BYTE_RESP = 0xBB

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(1)  # wait for sensor to be ready

# Compose read request
buf_out = bytearray([COM_START_BYTE_WR, COM_READ, CHIP_ID_ADDR, 1])
ser.write(buf_out)

# Read response (should be 0xBB, 0x01, 0xA0)
resp = ser.read(3)
print("Raw response:", list(resp))

ser.close()
