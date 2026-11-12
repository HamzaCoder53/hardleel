import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.137.239', username='hardleel', password='hardleel', timeout=5)

script = """
echo "=== CHRONY / NTP / TIMESYNCD ==="
systemctl is-active chrony 2>/dev/null || echo "chrony inactive/not installed"
systemctl is-active systemd-timesyncd 2>/dev/null || echo "timesyncd status checked"
timedatectl

echo "=== SERIAL BY-ID ==="
ls -la /dev/serial/by-id/ 2>/dev/null

echo "=== ARDUINO NANO RAW STREAM SAMPLE ==="
# Read 5 lines directly from Arduino serial port if available
ARDUINO_PORT=$(ls /dev/serial/by-id/usb-Arduino* 2>/dev/null | head -n 1)
if [ -n "$ARDUINO_PORT" ]; then
    echo "Found Arduino on $ARDUINO_PORT"
    python3 -c "
import serial, time
try:
    s = serial.Serial('$ARDUINO_PORT', 115200, timeout=2)
    time.sleep(1)
    for _ in range(10):
        line = s.readline().decode('utf-8', errors='ignore').strip()
        if line: print(line)
    s.close()
except Exception as e:
    print('Serial read error:', e)
"
else
    echo "Arduino port not found in /dev/serial/by-id"
fi

echo "=== PICO RAW STREAM / VERSION / TEL ==="
PICO_PORT=$(ls /dev/serial/by-id/usb-Raspberry_Pi_Pico* 2>/dev/null | head -n 1)
if [ -n "$PICO_PORT" ]; then
    echo "Found Pico on $PICO_PORT"
    python3 -c "
import serial, time
try:
    s = serial.Serial('$PICO_PORT', 115200, timeout=1)
    s.write(b'?\\n')
    time.sleep(0.2)
    s.write(b'STAT\\n')
    time.sleep(0.2)
    for _ in range(10):
        line = s.readline().decode('utf-8', errors='ignore').strip()
        if line: print(line)
    s.close()
except Exception as e:
    print('Pico read error:', e)
"
else
    echo "Pico not found in /dev/serial/by-id"
fi
"""

stdin, stdout, stderr = ssh.exec_command(f"bash -c '{script}'")
print(stdout.read().decode())
err = stderr.read().decode()
if err:
    print("ERR:", err)

ssh.close()
