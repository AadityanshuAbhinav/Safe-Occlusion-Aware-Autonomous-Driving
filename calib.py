#!/usr/bin/env python3
"""
Steering Calibration Helper

- Listens on CAN bus (socketcan).
- Prints raw front/rear steer values continuously.
- Use it to measure offsets (straight wheels) and full-lock values.
"""

import can
import os
import struct
import time

CAN_CHANNEL = 'can1' if os.path.exists('/sys/class/net/can1') else 'can0'
ID_STEER_FB = 0x532

def s16_le(data, offset):
    return struct.unpack_from('<h', data, offset)[0]

def u8(data, offset):
    return data[offset]

def bits(val, start, length):
    return (val >> start) & ((1 << length) - 1)

def decode_steer_fb(data):
    b0 = u8(data, 0)
    raw_front = s16_le(data, 1)
    raw_rear  = s16_le(data, 3)
    return {
        'raw_front': raw_front,
        'raw_rear': raw_rear,
        'steer_mode': bits(b0, 4, 4),
    }

if __name__ == "__main__":
    bus = can.interface.Bus(interface='socketcan', channel=CAN_CHANNEL)
    print("Listening on CAN bus for steer frames (0x532)...")
    print("Press Ctrl+C to stop.\n")

    try:
        count = 0
        while True:
            msg = bus.recv(timeout=0.1)
            if msg is None or msg.arbitration_id != ID_STEER_FB:
                continue
            d = decode_steer_fb(bytes(msg.data))
            print(f"RAW front={d['raw_front']}, RAW rear={d['raw_rear']}, mode={d['steer_mode']}")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        bus.shutdown()
