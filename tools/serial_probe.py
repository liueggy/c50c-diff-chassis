#!/usr/bin/env python3
"""Minimal serial probe: open a port and dump whatever bytes arrive.

Use it to check whether the board is transmitting at all, independent of any
serial-assistant tool. Optionally keeps DTR/RTS de-asserted, which helps on
boards whose CH9102 has an auto-download circuit wired to NRST/BOOT0 (some
tools hold the MCU in reset when they open the port).

Usage:
    python tools/serial_probe.py --port COM3
    python tools/serial_probe.py --port COM3 --no-dtr --seconds 5
"""
import argparse
import time


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--no-dtr", action="store_true",
                    help="keep DTR/RTS de-asserted before reading")
    args = ap.parse_args()

    try:
        import serial
    except ImportError:
        raise SystemExit("pip install pyserial")

    try:
        s = serial.Serial(args.port, args.baud, bytesize=8, parity="N",
                          stopbits=1, timeout=0.2)
    except Exception as e:
        raise SystemExit("open failed: %s  (is another tool holding the port?)" % e)

    if args.no_dtr:
        try:
            s.dtr = False
            s.rts = False
        except Exception:
            pass
        time.sleep(0.3)

    print("reading %s @ %d for %.1fs ..." % (args.port, args.baud, args.seconds))
    end = time.time() + args.seconds
    total = 0
    while time.time() < end:
        d = s.read(256)
        if d:
            total += len(d)
            print("+%d bytes: %s" % (len(d), d.hex(" ")))
    s.close()
    print("total bytes:", total)
    if total == 0:
        print("NOTHING received. Check: board powered? latest firmware flashed? "
              "DTR/RTS holding reset? (try --no-dtr) right port?")


if __name__ == "__main__":
    main()
