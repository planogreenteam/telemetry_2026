#!/usr/bin/env python3
"""Bench tool: talk to the EG4 LL-S over RS485 and dump its registers.

Use this BEFORE trusting the telemetry to confirm the wiring, the address
and the register map in BMS/bms_normalizer.py.

    # Which address answers? (tries 1..16)
    python -m BMS.bms_probe --port /dev/serial/by-id/usb-... --scan

    # Raw register dump + decoded view for one address
    python -m BMS.bms_probe --port /dev/serial/by-id/usb-... --address 1

Compare the decoded SOC / pack voltage / cells with the battery's display
or the EG4 app. If a value is in the wrong register, edit REG in
BMS/bms_normalizer.py.
"""

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from BMS.bms_normalizer import (  # noqa: E402
    REG, REGISTER_BLOCK_COUNT, REGISTER_BLOCK_START, normalize_bms_frame,
)
from BMS.bms_reader import EG4ModbusReader, ModbusError  # noqa: E402

# Read the wider dump in chunks: some BMS firmware rejects big requests.
DUMP_CHUNK = 32

_LABELS = {addr: name for name, addr in REG.items()}
for _i in range(16):
    _LABELS[REG["cell_first"] + _i] = f"cell {_i + 1}"


def scan(reader, first, last):
    found = []
    for address in range(first, last + 1):
        reader.address = address
        try:
            reader.read_registers(0, 1)
            print(f"  address {address}: ANSWERED")
            found.append(address)
        except ModbusError as exc:
            print(f"  address {address}: {exc}")
    if not found:
        print("\nNo answer on any address. Check A/B wiring (try swapping), "
              "the port, the baud rate, and that the battery is on.")
    return found


def dump(reader, count):
    print(f"\nRaw registers 0..{count - 1} from address {reader.address}:")
    print(f"{'reg':>4}  {'hex':>6}  {'u16':>6}  {'s16':>6}  label")
    for start in range(0, count, DUMP_CHUNK):
        n = min(DUMP_CHUNK, count - start)
        try:
            regs = reader.read_registers(start, n)
        except ModbusError as exc:
            print(f"  regs {start}..{start + n - 1}: {exc}")
            continue
        for offset, value in enumerate(regs):
            reg = start + offset
            s16 = value - 0x10000 if value & 0x8000 else value
            ascii_pair = bytes([value >> 8, value & 0xFF])
            text = ascii_pair.decode("ascii") if all(32 <= b < 127 for b in ascii_pair) else ""
            label = _LABELS.get(reg, "")
            if text:
                label = f"{label}  '{text}'".strip()
            print(f"{reg:>4}  0x{value:04X}  {value:>6}  {s16:>6}  {label}")


def decoded(reader):
    regs = reader.read_registers(REGISTER_BLOCK_START, REGISTER_BLOCK_COUNT)
    reading = normalize_bms_frame({"registers": regs}, device_id=0)
    print("\nDecoded with the current register map:")
    for key, value in reading["fields"].items():
        print(f"  {key:<20} {value}")


def main(argv=None):
    parser = argparse.ArgumentParser(description="EG4 LL-S RS485/Modbus probe")
    parser.add_argument("--port", required=True, help="USB-RS485 adapter serial device")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--address", type=int, default=1, help="Battery DIP address")
    parser.add_argument("--scan", action="store_true", help="Try addresses 1..16 and stop")
    parser.add_argument("--count", type=int, default=128, help="Registers to dump")
    args = parser.parse_args(argv)

    reader = EG4ModbusReader(args.port, baud=args.baud, address=args.address)
    try:
        if args.scan:
            scan(reader, 1, 16)
            return
        dump(reader, args.count)
        try:
            decoded(reader)
        except ModbusError as exc:
            print(f"\nDecoded view unavailable: {exc}")
    finally:
        reader.close()


if __name__ == "__main__":
    main()
