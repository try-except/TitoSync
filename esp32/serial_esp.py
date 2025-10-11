import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
for i in range(10):
    message = f"0,5,5,10\n"
    ser.write(message.encode('ascii'))
    time.sleep(1)

time.sleep(0.1)  # small delay to ensure ESP reads it
ser.close()
