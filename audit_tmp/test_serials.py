import paramiko

ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect('192.168.137.239', username='hardleel', password='hardleel', timeout=5)

code = '''
import serial, time
try:
    s = serial.Serial("/dev/serial/by-id/usb-Arduino_Nano_33_BLE_37508B91BB6D52DF-if00", 115200, timeout=2)
    time.sleep(1)
    for _ in range(10):
        l = s.readline().decode("utf-8", errors="ignore").strip()
        if l: print("NANO:", l)
    s.close()
except Exception as e:
    print("Nano error:", e)

try:
    s = serial.Serial("/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6614104033D2039-if00", 115200, timeout=1)
    s.write(b"?\\nSTAT\\n")
    time.sleep(0.5)
    for _ in range(10):
        l = s.readline().decode("utf-8", errors="ignore").strip()
        if l: print("PICO:", l)
    s.close()
except Exception as e:
    print("Pico error:", e)
'''

stdin, stdout, stderr = ssh.exec_command(f'python3 -c "{code}"')
print(stdout.read().decode())
err = stderr.read().decode()
if err: print("ERR:", err)

ssh.close()
