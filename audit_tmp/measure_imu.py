import serial, time

s = serial.Serial('/dev/serial/by-id/usb-Arduino_Nano_33_BLE_37508B91BB6D52DF-if00', 115200, timeout=1)
time.sleep(1)
t0 = time.time()
count = 0
accel = [0.0, 0.0, 0.0]
gyro = [0.0, 0.0, 0.0]

while time.time() - t0 < 5.0:
    l = s.readline().decode('utf-8', errors='ignore').strip()
    parts = l.split(',')
    if len(parts) == 7 and parts[0] == 'I':
        count += 1
        ax, ay, az, gx, gy, gz = [float(x) for x in parts[1:]]
        accel[0] += ax; accel[1] += ay; accel[2] += az
        gyro[0] += gx; gyro[1] += gy; gyro[2] += gz

s.close()
dur = time.time() - t0
rate = count / dur
print(f'Captured {count} samples in {dur:.2f} s ({rate:.1f} Hz)')
if count > 0:
    print(f'Mean ax: {accel[0]/count:.4f} g ({accel[0]/count*9.80665:.3f} m/s^2)')
    print(f'Mean ay: {accel[1]/count:.4f} g ({accel[1]/count*9.80665:.3f} m/s^2)')
    print(f'Mean az: {accel[2]/count:.4f} g ({accel[2]/count*9.80665:.3f} m/s^2)')
    print(f'Mean gx: {gyro[0]/count:.4f} deg/s')
    print(f'Mean gy: {gyro[1]/count:.4f} deg/s')
    print(f'Mean gz: {gyro[2]/count:.4f} deg/s')
