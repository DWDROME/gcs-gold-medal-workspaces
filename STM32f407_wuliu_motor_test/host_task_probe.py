#!/usr/bin/env python3
import argparse
import sys
import time

import serial


def build_pkt(payload: str) -> bytes:
    data = payload.encode("ascii", errors="strict")
    if len(data) > 9:
        raise ValueError("payload too long, max 9 ascii chars")
    data = data.ljust(9, b"\x00")
    return bytes([0xFF]) + data + bytes([0xFE])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--send", required=True, help="ascii payload, max 9 chars")
    ap.add_argument("--wait", type=float, default=0.3, help="seconds to wait before read")
    ap.add_argument("--read", type=int, default=64, help="max bytes to read")
    args = ap.parse_args()

    pkt = build_pkt(args.send)
    print(f"port={args.port} baud={args.baud}")
    print(f"send={pkt!r}")

    ser = serial.Serial(args.port, args.baud, timeout=0.2)
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ser.write(pkt)
        ser.flush()
        time.sleep(args.wait)
        data = ser.read(args.read)
        print(f"recv={data!r}")
    finally:
        ser.close()

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"err={exc}", file=sys.stderr)
        raise
