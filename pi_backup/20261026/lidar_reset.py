import serial, sys, time

# Stop scanning, reset the LiDAR, then ask for device info, retrying while it reboots.
# Prints OK <hex> and exits 0 on a valid reply, otherwise prints "no valid reply" and exits 1.
import glob
# port: argv[1], else the FTDI by-id path (ttyUSB0/1 depends on plug order)
PORT = sys.argv[1] if len(sys.argv) > 1 else (sorted(glob.glob("/dev/serial/by-id/usb-FTDI_*")) or ["/dev/ttyUSB0"])[0]
s = serial.Serial(PORT, 115200, timeout=1)
s.dtr = False   # motor enabled: since 22 Oct the A1 only answers with DTR low
s.write(bytes([0xA5, 0x25]))   # stop scanning
time.sleep(0.5)
s.write(bytes([0xA5, 0x40]))   # reset the device
data = b""
for attempt in range(6):
    time.sleep(2.5)
    s.reset_input_buffer()
    s.write(bytes([0xA5, 0x50]))   # get device info
    time.sleep(0.5)
    data = s.read(27)
    if data.startswith(b"\xa5\x5a"):
        print("OK", data.hex())
        s.close()
        sys.exit(0)
print("no valid reply", data.hex())
s.close()
sys.exit(1)
