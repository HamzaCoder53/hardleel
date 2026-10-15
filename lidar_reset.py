import serial, time

s = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
s.write(bytes([0xA5, 0x25]))   # stop scanning
time.sleep(0.5)
s.write(bytes([0xA5, 0x40]))   # reset the device
time.sleep(2.5)
s.reset_input_buffer()
s.write(bytes([0xA5, 0x50]))   # get device info
time.sleep(0.5)
data = s.read(27)
print('OK' if data.startswith(b'\xa5\x5a') else 'no valid reply', data.hex())
s.close()
