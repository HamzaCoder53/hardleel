import serial, time

s = serial.Serial('/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0', 115200, timeout=1)
time.sleep(0.5)

# Start scan command
s.write(bytes([0xA5, 0x20]))
time.sleep(0.5)
resp = s.read(7)
print('Scan response header:', resp.hex())

# Read scan points
t0 = time.time()
scans = 0
points = 0
last_angle = 0
while time.time() - t0 < 5.0:
    b = s.read(5)
    if len(b) == 5:
        s0 = b[0] & 0x01
        s1 = (b[0] >> 1) & 0x01
        check = b[1] & 0x01
        if s0 != s1 and check == 1:
            quality = b[0] >> 2
            angle_q6 = ((b[1] >> 1) | (b[2] << 7))
            angle_deg = angle_q6 / 64.0
            distance_q2 = (b[3] | (b[4] << 8))
            dist_mm = distance_q2 / 4.0
            points += 1
            if s0 == 1: # start of a new scan revolution
                scans += 1

# Stop scan
s.write(bytes([0xA5, 0x25]))
s.close()
dur = time.time() - t0
print(f'Captured {scans} full scan revolutions ({points} points) in {dur:.2f} s')
if scans > 0:
    print(f'Actual Scan Rate: {scans/dur:.2f} Hz')
    print(f'Average samples/rev: {points/scans:.1f}')
