#!/usr/bin/env python3
"""Host-side serial driver for the Chassis differential UART control.

Sends the 11-byte motion frame on the control board's USART3 or UART4 Bluetooth link.

Frame (all bytes, 115200 8N1):
    [0] 0x7B                              frame header
    [1] 0x00 speed / 0x10 release          release requires zero payload
    [2] 0x00                              reserved (must be 0)
    [3] vx_hi  [4] vx_lo                  linear speed, signed BE, mm/s
    [5] 0x00   [6] 0x00                   lateral speed (must be 0)
    [7] wz_hi  [8] wz_lo                  angular rate, signed BE, mrad/s
    [9] XOR of bytes[0..8]                checksum
    [10] 0x7D                             frame tail

Sign convention (differential drive, A=left / B=right):
    vx > 0 forward, vx < 0 backward
    wz > 0 turn left (CCW), wz < 0 turn right (CW)
Ranges: |vx| <= 700 mm/s, |wz| <= 1000 mrad/s. Out-of-range frames are
rejected by the firmware (and disarm it), so keep commands in range.

Safety/behaviour of the firmware this talks to:
  * requires a zero frame (vx=0, wz=0) first to arm;
  * stops if no valid frame arrives within 500 ms;
  * ignores serial for ~10 s after power-on (CONTROL_DELAY).
This script therefore sends a zero frame first, streams the requested
command at ~20 Hz, and sends zero on exit.

Examples:
    python tools/serial_drive.py --vx 200                 # forward 200 mm/s
    python tools/serial_drive.py --vx 200 --wz 300 --duration 3
    python tools/serial_drive.py --wz -500                # spin right
    python tools/serial_drive.py --interactive            # WASD teleop
    python tools/serial_drive.py --dry-run --vx 200       # print frames only

Requires pyserial:  pip install pyserial
"""

import argparse
import struct
import sys
import time

FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
MAX_LINEAR_MM_S = 700
MAX_ANGULAR_MRAD_S = 1000


def build_frame(vx_mm_s, wz_mrad_s):
    """Return the 11-byte command frame for vx (mm/s) and wz (mrad/s)."""
    body = bytes([FRAME_HEADER, 0x00, 0x00])
    body += struct.pack(">h", int(round(vx_mm_s)))
    body += bytes([0x00, 0x00])
    body += struct.pack(">h", int(round(wz_mrad_s)))
    checksum = 0
    for b in body:
        checksum ^= b
    return body + bytes([checksum, FRAME_TAIL])


def build_release_frame():
    """Release this link's ownership; never follow it with zero heartbeats."""
    frame = bytearray(build_frame(0, 0))
    frame[1] = 0x10
    frame[9] ^= 0x10
    return bytes(frame)


def clamp_command(vx, wz):
    """Clamp into the firmware's accepted range (avoids a rejected frame)."""
    vx = max(-MAX_LINEAR_MM_S, min(MAX_LINEAR_MM_S, vx))
    wz = max(-MAX_ANGULAR_MRAD_S, min(MAX_ANGULAR_MRAD_S, wz))
    return vx, wz


def open_port(port, baud):
    try:
        import serial
    except ImportError:
        sys.exit("pyserial is required:  pip install pyserial")
    try:
        ser = serial.Serial(port=None, baudrate=baud, bytesize=8, parity="N", stopbits=1,
                            timeout=0, write_timeout=0.1)
        ser.dtr = False
        ser.rts = False
        ser.port = port
        ser.open()
        return ser
    except Exception as exc:  # noqa: BLE001 - report the OS error verbatim
        sys.exit("Cannot open %s @ %d: %s" % (port, baud, exc))


def stream(ser, vx, wz, duration, rate, verbose):
    """Send zero first, then stream (vx, wz) until duration/timeout/quit."""
    period = 1.0 / rate
    deadline = None if duration <= 0 else time.monotonic() + duration
    zero = build_frame(0, 0)
    cmd = build_frame(vx, wz)

    # Arm: a short burst of zero frames so the parser enters ACTIVE.
    for _ in range(5):
        ser.write(zero)
        time.sleep(period)

    try:
        while deadline is None or time.monotonic() < deadline:
            ser.write(cmd)
            if verbose:
                sys.stdout.write("sent vx=%+d mm/s wz=%+d mrad/s  %s   \r"
                                 % (vx, wz, cmd.hex(" ")))
                sys.stdout.flush()
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):          # Stop: return to zero.
            ser.write(zero)
            time.sleep(period)
        if verbose:
            sys.stdout.write("\nstopped\n")


def interactive(ser, rate, verbose):
    """WASD teleop. w/s adjust vx, a/d adjust wz, space stops, q quits."""
    try:
        import msvcrt  # Windows
    except ImportError:
        sys.exit("--interactive needs Windows (msvcrt); use --vx/--wz instead.")

    vx = wz = 0
    period = 1.0 / rate
    zero = build_frame(0, 0)
    for _ in range(5):              # Arm with zero frames.
        ser.write(zero)
        time.sleep(period)

    print("WASD: w/s vx +-50, a/d wz +-100, space=stop, q=quit")
    print("  vx(+-700 mm/s, fwd/back)   wz(+-1000 mrad/s, left/right)")
    try:
        while True:
            if msvcrt.kbhit():
                key = msvcrt.getch().decode("latin-1", "ignore").lower()
                if key == "w":
                    vx += 50
                elif key == "s":
                    vx -= 50
                elif key == "d":
                    wz -= 100
                elif key == "a":
                    wz += 100
                elif key == " ":
                    vx = wz = 0
                elif key == "q":
                    break
                vx, wz = clamp_command(vx, wz)
                print("  -> vx=%+d mm/s  wz=%+d mrad/s   " % (vx, wz))
            ser.write(build_frame(vx, wz))
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            ser.write(zero)
            time.sleep(period)
        print("stopped")


def main():
    ap = argparse.ArgumentParser(description="Chassis serial velocity control")
    ap.add_argument("--port", default="COM3", help="serial port (default COM3)")
    ap.add_argument("--baud", type=int, help="default 115200 for UART3, 9600 for Bluetooth")
    ap.add_argument("--source", choices=["uart3", "bluetooth"], default="uart3")
    ap.add_argument("--release", action="store_true", help="release this control link and exit")
    ap.add_argument("--vx", type=float, default=0.0, help="linear mm/s, +fwd/-back")
    ap.add_argument("--wz", type=float, default=0.0, help="angular mrad/s, +left/-right")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="seconds to hold the command (0 = until Ctrl-C)")
    ap.add_argument("--rate", type=float, default=20.0, help="send rate Hz (default 20)")
    ap.add_argument("--interactive", action="store_true", help="WASD teleop")
    ap.add_argument("--dry-run", action="store_true", help="print frames, send nothing")
    ap.add_argument("--verbose", action="store_true", help="print every frame")
    args = ap.parse_args()

    if args.baud is None:
        args.baud = 9600 if args.source == "bluetooth" else 115200
    if args.release and args.interactive:
        ap.error("--release and --interactive cannot be combined")
    vx, wz = clamp_command(args.vx, args.wz)

    if args.dry_run:
        if args.release:
            print("release  :", build_release_frame().hex(" "))
            return
        print("zero arm :", build_frame(0, 0).hex(" "))
        print("command  :", build_frame(vx, wz).hex(" "),
              "(vx=%+d mm/s, wz=%+d mrad/s)" % (vx, wz))
        return

    ser = open_port(args.port, args.baud)
    print("opened %s @ %d; robot ignores serial for ~10 s after power-on."
          % (args.port, args.baud))
    try:
        if args.release:
            for _ in range(3):
                ser.write(build_release_frame())
                time.sleep(0.05)
            print("release requested; no zero heartbeat follows")
        elif args.interactive:
            interactive(ser, args.rate, args.verbose)
        else:
            print("sending vx=%+d mm/s wz=%+d mrad/s for %s"
                  % (vx, wz, "Ctrl-C" if args.duration <= 0 else "%.1fs" % args.duration))
            stream(ser, vx, wz, args.duration, args.rate, args.verbose)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
