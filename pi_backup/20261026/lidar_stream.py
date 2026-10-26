import serial, time
s = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
s.dtr = True; time.sleep(1.5); s.reset_input_buffer()
s.write(bytes([0xA5, 0x25])); time.sleep(0.3); s.reset_input_buffer()
s.write(bytes([0xA5, 0x52])); time.sleep(0.3); h = s.read(10); print("health", h.hex())
s.write(bytes([0xA5, 0x20])); time.sleep(0.2)
hdr = s.read(7); print("scan hdr", hdr.hex())
t0 = time.time(); n = 0
while time.time() - t0 < 3:
    n += len(s.read(500))
print("bytes in 3s:", n, "(expect ~ 5 Hz x 360 pts x 5 B = ~27000 when scanning)")
s.write(bytes([0xA5, 0x25])); s.close()
