#!/usr/bin/env python3
"""RPLidar <-> Portenta UART bridge via PC USB.

Bridge waits for Portenta to finish booting (WiFi + micro-ROS),
then starts RPLidar and relays bidirectionally.
"""

import serial
import threading
import sys
import time

RPLIDAR_PORT  = "/dev/rplidar"
PORTENTA_PORT = "/dev/portenta"
RPLIDAR_BAUD  = 460800  # C1: 460800, A1: 115200, A2: 256000
MOTOR_PWM     = 660
BOOT_WAIT     = 20      # seconds to wait for Portenta setup (WiFi + micro-ROS)

def forward(src, dst, label, tap=False):
    try:
        while True:
            data = src.read(min(src.in_waiting or 1, 64))
            if data:
                dst.write(data)
                dst.flush()
                if tap:
                    try:
                        text = data.decode("ascii", errors="ignore")
                        printable = "".join(c for c in text if c.isprintable() or c in "\r\n")
                        if printable:
                            sys.stdout.write(printable)
                            sys.stdout.flush()
                    except Exception:
                        pass
    except Exception as e:
        print(f"[{label}] stopped: {e}")

def init_rplidar(rp):
    """Stop, start motor, start scan."""
    rp.write(bytes([0xA5, 0x25]))
    time.sleep(0.1)
    rp.reset_input_buffer()

    lo = MOTOR_PWM & 0xFF
    hi = (MOTOR_PWM >> 8) & 0xFF
    chk = 0 ^ 0xA5 ^ 0xF0 ^ 0x02 ^ lo ^ hi
    rp.write(bytes([0xA5, 0xF0, 0x02, lo, hi, chk & 0xFF]))
    print("Motor PWM=%d, waiting for spin-up..." % MOTOR_PWM)
    time.sleep(2)
    rp.reset_input_buffer()

    rp.write(bytes([0xA5, 0x20]))
    time.sleep(0.5)
    print("Scan started, %d bytes buffered" % rp.in_waiting)

def main():
    baud = int(sys.argv[1]) if len(sys.argv) > 1 else RPLIDAR_BAUD

    rplidar = serial.Serial(RPLIDAR_PORT, baud, timeout=0.01)
    rplidar.dtr = False

    print("Bridge: %s (%d) <-> %s" % (RPLIDAR_PORT, baud, PORTENTA_PORT))
    print("Waiting %ds for Portenta boot..." % BOOT_WAIT)
    time.sleep(BOOT_WAIT)

    portenta = serial.Serial()
    portenta.port = PORTENTA_PORT
    portenta.baudrate = baud
    portenta.timeout = 0.01
    portenta.dtr = False
    portenta.rts = False
    portenta.open()

    # Drain any buffered output from Portenta
    time.sleep(0.1)
    if portenta.in_waiting:
        portenta.read(portenta.in_waiting)

    init_rplidar(rplidar)

    print("Relaying... Ctrl+C to stop")

    t1 = threading.Thread(target=forward, args=(rplidar, portenta, "lidar->mcu"), daemon=True)
    t2 = threading.Thread(target=forward, args=(portenta, rplidar, "mcu->lidar", True), daemon=True)
    t1.start()
    t2.start()

    try:
        t1.join()
    except KeyboardInterrupt:
        rplidar.write(bytes([0xA5, 0x25]))
        print("\nStopped")

if __name__ == "__main__":
    main()
