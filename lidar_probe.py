import serial, time

for baud in (115200, 256000):
    for dtr in (False, True):
        s = serial.Serial('/dev/ttyUSB0', baud, timeout=0.3)
        s.dtr = dtr
        time.sleep(1.0)
        s.reset_input_buffer()

        s.write(bytes([0xA5, 0x40]))            # reset: the A1 prints a text banner when it reboots
        t, banner = time.time(), b''
        while time.time() - t < 3.0:
            banner += s.read(200)

        s.reset_input_buffer()
        s.write(bytes([0xA5, 0x50]))            # get device info
        time.sleep(0.5)
        info = s.read(27)

        print(f'baud={baud} dtr={dtr}: banner {len(banner)} bytes '
              f'{banner[:40]!r} | info {info.hex() if info else "none"}')
        s.close()
